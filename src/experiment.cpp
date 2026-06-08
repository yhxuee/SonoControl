#include "experiment.hpp"

#include "pid.hpp"
#include "protocol.hpp"
#include "serial_port.hpp"
#include "udp_socket.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace sonocontrol {

namespace {
constexpr double COM_GAP_S = 0.1;

std::string fmt_double(double v, int precision) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

std::string left_width(const std::string& s, int width) {
    std::ostringstream os;
    os << std::left << std::setw(width) << s;
    return os.str();
}

int retry_attempts(const Config& c) {
    return c.communication_retry ? std::max(1, c.communication_retry_attempts) : 1;
}

int backoff_ms_for_attempt(const Config& c, int attempt) {
    const int base = std::max(20, c.communication_retry_initial_backoff_ms);
    return base * (1 << std::min(attempt, 4));
}

// Sleep that wakes early when a stop flag is set. Long retry backoffs
// (hundreds of ms to several seconds) would otherwise make EMERGENCY STOP
// feel unresponsive — the worker thread can't observe stop_flag_ while it
// is inside a long sleep_for. Chunking into 25 ms slices keeps total sleep
// time accurate to ±25 ms while letting the worker react quickly.
inline void interruptible_sleep_ms(int ms, const std::atomic<bool>& stop_flag) {
    constexpr int kSlice = 25;
    int remaining = std::max(0, ms);
    while (remaining > 0 && !stop_flag.load(std::memory_order_acquire)) {
        const int step = std::min(kSlice, remaining);
        std::this_thread::sleep_for(std::chrono::milliseconds(step));
        remaining -= step;
    }
}


} // namespace

ExperimentRunner::ExperimentRunner(Config config,
                                   std::unique_ptr<ITemperatureSensor> sensor,
                                   ExperimentLogger& logger,
                                   ConsoleCallback console)
    : config_(std::move(config)), sensor_(std::move(sensor)), logger_(logger) {
    callbacks_.console = std::move(console);
}

ExperimentRunner::ExperimentRunner(Config config,
                                   std::unique_ptr<ITemperatureSensor> sensor,
                                   ExperimentLogger& logger,
                                   ExperimentCallbacks callbacks)
    : config_(std::move(config)), sensor_(std::move(sensor)), logger_(logger), callbacks_(std::move(callbacks)) {}

void ExperimentRunner::request_stop() {
    // Latch FIRST: stop_flag_ may be temporarily cleared by send_stop() to
    // force STOP packet delivery. If request_stop fires during that window,
    // stop_flag_=true would be overwritten back to false on the restore — but
    // manual_stop_latched_ is monotonic, so end-of-run code selection still
    // sees the operator intent.
    manual_stop_latched_ = true;
    stop_flag_ = true;
}

void ExperimentRunner::force_stop() {
    manual_stop_latched_ = true;
    // Order matters: set the flag first so that any code that recovers from a
    // cancelled I/O sees the stop request and bails out instead of retrying.
    stop_flag_ = true;
    // Unblock anything currently inside the OS kernel. cancel_io is idempotent
    // and safe to call when the handle/socket is closed.
    try { ultrasound_serial_.cancel_io(); } catch (...) {}
    try { udp_sender_.cancel_io(); } catch (...) {}
}

void ExperimentRunner::watchdog_stop() {
    // Identical unblock-and-stop mechanism to force_stop(), but latches a
    // watchdog stop rather than a manual one. Deliberately does NOT touch
    // manual_stop_latched_: a stall the watchdog had to break is reported as an
    // automatic comms-stall stop (code 4), not as an operator stop (code 3).
    watchdog_stop_latched_ = true;
    stop_flag_ = true;
    try { ultrasound_serial_.cancel_io(); } catch (...) {}
    try { udp_sender_.cancel_io(); } catch (...) {}
}

ActiveParams ExperimentRunner::initial_params_from_config(const Config& c) {
    return ActiveParams{c.amplitude, c.cfreq_hz, c.prf_hz, c.duty_cycle,
                        c.duration_ms, c.interval_time_s, c.wave_shape};
}

int ExperimentRunner::run() {
    stop_flag_ = false;
    manual_stop_latched_ = false;
    watchdog_stop_latched_ = false;
    PIDController pid(config_.pid_kp, config_.pid_ki, config_.pid_kd);

    const ActiveParams initial_params = initial_params_from_config(config_);
    ActiveParams current_params = initial_params;

    if (config_.pid_enabled && config_.length_mode == LengthMode::RepeatingCycles) {
        throw std::runtime_error("Repeating cycles mode is not allowed when PID is enabled. Use total duration or hold-after-target mode.");
    }
    if (config_.length_mode == LengthMode::HoldAfterTarget && !config_.temperature_enabled) {
        throw std::runtime_error("Hold-after-target mode requires temperature monitoring.");
    }

    const bool cycling_enabled = config_.use_cycling && config_.length_mode == LengthMode::TotalDuration;
    const double total_s = (config_.length_mode == LengthMode::TotalDuration)
        ? config_.total_duration_mins * 60.0
        : ((config_.length_mode == LengthMode::RepeatingCycles)
            ? static_cast<double>(config_.repeating) * config_.interval_time_s
            : 0.0);
    const double hold_after_target_s = std::max(0.0, config_.hold_after_target_mins * 60.0);
    const double target_tolerance = std::max(0.0, config_.target_tolerance_c);
    bool target_hold_started = false;
    auto target_hold_start = std::chrono::steady_clock::now();
    const double sampling_period = 1.0 / std::max(0.1, config_.sampling_rate_hz);

    CyclePhase phase = CyclePhase::Heating;
    auto phase_start = std::chrono::steady_clock::now();
    const auto experiment_start = std::chrono::steady_clock::now();
    auto last_send_time = experiment_start;
    int cycle_count = 0;
    auto next_sample_time = experiment_start;
    int invalid_temp_samples = 0;
    int over_cutoff_samples = 0;
    auto last_over_cutoff_time = experiment_start - std::chrono::seconds(3600);
    bool temp_fallback_active = false;

    bool predictor_initialized = false;
    double filtered_control_temp = 0.0;
    // (t_rel_s, raw_temp) sliding window for the least-squares dT/dt fit. The
    // window width is config_.temp_rate_window_s (default 30 s, GUI-editable).
    std::deque<std::pair<double, double>> rate_window;

    logger_.start_session(initial_params, config_);
    // Seed the per-sample state columns. Phase tracks the experiment state
    // machine: HEATING for plain runs and PID heating, WAIT_TARGET while
    // HoldAfterTarget mode is climbing to setpoint. Setpoint is populated
    // whenever a control target exists (PID enabled, or HoldAfterTarget).
    // Demand and US active follow the first transmit / PID compute.
    if (config_.length_mode == LengthMode::HoldAfterTarget) {
        logger_.set_phase("WAIT_TARGET");
        logger_.set_setpoint(config_.pid_setpoint);
    } else if (config_.pid_enabled) {
        logger_.set_phase("HEATING");
        logger_.set_setpoint(config_.pid_setpoint);
    } else {
        logger_.set_phase("HEATING");
        logger_.set_setpoint(std::nullopt);
    }
    logger_.set_pid_demand(std::nullopt);
    logger_.set_ultrasound_active(false);
    log_console(std::string("Experiment started. Length mode=") + to_string(config_.length_mode) +
                ", planned=" + (config_.length_mode == LengthMode::HoldAfterTarget
                    ? ("hold " + fmt_double(hold_after_target_s, 1) + "s after target")
                    : (fmt_double(total_s, 1) + "s")) +
                ", Interval=" + fmt_double(initial_params.interval_time_s, 1) + "s");
    log_console("Streaming log: " + logger_.csv_path().string());
    log_console("Cycle: HEATING");
    if (!config_.temperature_enabled) {
        log_console("Temperature monitoring disabled. HH806AU, PID feedback, and cutoff checks are disabled.");
    } else if (config_.pid_enabled) {
        log_console("Temperature monitoring enabled. HH806AU is required because PID feedback is active.");
    } else if (config_.length_mode == LengthMode::HoldAfterTarget) {
        log_console("Temperature monitoring enabled. HH806AU is required because hold-after-target mode uses temperature to start the hold timer.");
    } else {
        log_console("Temperature monitoring enabled without PID. HH806AU is optional preflight; PID feedback is inactive.");
    }
    log_console("Communication retry/backoff: " + std::to_string(retry_attempts(config_)) +
                " attempt(s), initial backoff=" + std::to_string(std::max(20, config_.communication_retry_initial_backoff_ms)) + " ms");
    if (config_.pid_enabled) {
        log_console("PID predictive thermal const=" + fmt_double(std::max(0.0, config_.pid_prediction_tau_s), 1) +
                    "s, horizon=" + fmt_double(std::max(0.0, config_.pid_prediction_horizon_s), 1) +
                    "s (0=interval); model: T_future=T+tau*dT/dt*(1-exp(-horizon/tau)); "
                    "hardware updates still follow interval timing.");
        log_console("Configuration source: " + config_.config_source_type +
                    (config_.config_file_path.empty() ? std::string() : (" (" + config_.config_file_path + ")")));
    }

    if (!config_.simulate_us) {
        if (config_.persistent_com3) {
            ensure_ultrasound_serial_open();
            log_console("COM3 persistent connection opened: " + config_.com3_port);
        }
        if (!udp_sender_.open(4561)) {
            logger_.log_error("Cannot bind UDP source port 4561");
            throw std::runtime_error("Cannot bind UDP source port 4561");
        }
    }

    last_burst_valid_ = false;
    if (callbacks_.cycle) callbacks_.cycle("HEATING");

    transmit(current_params);
    last_send_time = std::chrono::steady_clock::now();

    while (!stop_flag_) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - experiment_start).count();
        double remaining = 0.0;
        if (config_.length_mode == LengthMode::TotalDuration) {
            remaining = std::max(0.0, total_s - elapsed);
            if (elapsed >= total_s) break;
        } else if (config_.length_mode == LengthMode::RepeatingCycles) {
            remaining = std::max(0.0, total_s - elapsed);
            if (cycle_count >= config_.repeating) break;
        } else {
            if (target_hold_started) {
                const double held = std::chrono::duration<double>(now - target_hold_start).count();
                remaining = std::max(0.0, hold_after_target_s - held);
                if (held >= hold_after_target_s) break;
            } else {
                remaining = hold_after_target_s;
            }
        }

        if (config_.temperature_enabled && now >= next_sample_time) {
            next_sample_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(sampling_period));
            const auto [t1, t2] = sensor_->read();

            std::vector<double> active;
            std::string active_channel = "none";
            bool used_fallback = false;
            if (config_.temp_channel == TempChannel::T1) {
                if (t1) {
                    active.push_back(*t1);
                    active_channel = "T1";
                } else if (config_.temp_channel_fallback && t2) {
                    active.push_back(*t2);
                    active_channel = "T2 fallback";
                    used_fallback = true;
                }
            } else if (config_.temp_channel == TempChannel::T2) {
                if (t2) {
                    active.push_back(*t2);
                    active_channel = "T2";
                } else if (config_.temp_channel_fallback && t1) {
                    active.push_back(*t1);
                    active_channel = "T1 fallback";
                    used_fallback = true;
                }
            } else {
                if (t1) active.push_back(*t1);
                if (t2) active.push_back(*t2);
                active_channel = active.size() == 2 ? "Average T1+T2" : (t1 ? "T1 only available" : (t2 ? "T2 only available" : "none"));
                used_fallback = active.size() == 1;
            }

            if (!active.empty()) {
                invalid_temp_samples = 0;
                double avg = 0.0;
                for (double v : active) avg += v;
                avg /= static_cast<double>(active.size());

                double control_temp_filtered = avg;
                double control_rate_c_per_s = 0.0;
                const double t_rel = std::chrono::duration<double>(now - experiment_start).count();
                if (!predictor_initialized) {
                    filtered_control_temp = avg;
                    predictor_initialized = true;
                } else {
                    // α=0.25 single-pole IIR smooths the temperature *level*
                    // (2 Hz samples, <~2 sample lag). The slope is estimated
                    // separately below so the two concerns stay decoupled.
                    constexpr double alpha = 0.25;
                    filtered_control_temp = filtered_control_temp + alpha * (avg - filtered_control_temp);
                }
                control_temp_filtered = filtered_control_temp;

                // dT/dt for the predictive model. A single-sample finite
                // difference at 2 Hz is dominated by probe quantisation: ±0.1 °C
                // of jitter over 0.5 s reads as ±0.2 °C/s — the entire clamp
                // range. So once enough samples have accumulated we fit a
                // least-squares line over a sliding window (temp_rate_window_s,
                // default 30 s) and take its slope: the regression averages out
                // per-sample noise while still tracking real trends, and it runs
                // on the *raw* channel average (not the EWMA) so the slope carries
                // no filter lag — the fit itself is the smoother. Three points is
                // the smallest window that actually smooths: a two-point
                // least-squares line is identical to a plain finite difference,
                // which is exactly the fallback used below before the window fills.
                const double kRateWindowSeconds = std::clamp(config_.temp_rate_window_s,
                    rate_window_floor_s(config_.sampling_rate_hz), kMaxRateWindowS);
                rate_window.emplace_back(t_rel, avg);
                while (rate_window.size() > 2 &&
                       t_rel - rate_window.front().first > kRateWindowSeconds) {
                    rate_window.pop_front();
                }
                const double window_span = rate_window.back().first - rate_window.front().first;
                // Smoothing needs both >=3 points and enough lever arm in t (the
                // configured window or 5 s, whichever is smaller) before its slope
                // is trustworthy. Until then — at start-up, or with a window too
                // small to hold three samples — fall back to a plain finite
                // difference over the span we do have instead of leaving the rate
                // at 0, so the predictor always gets a usable slope. The baseline
                // grows toward the warm-up span, so the fallback is noisiest only
                // for the first sample or two and then steadily smooths in.
                const double warmup_span = std::min(kRateWindowSeconds, 5.0);
                if (rate_window.size() >= 3 && window_span >= warmup_span) {
                    double sum_t = 0.0, sum_y = 0.0;
                    for (const auto& [t, y] : rate_window) { sum_t += t; sum_y += y; }
                    const double n = static_cast<double>(rate_window.size());
                    const double mean_t = sum_t / n, mean_y = sum_y / n;
                    double sxx = 0.0, sxy = 0.0;
                    for (const auto& [t, y] : rate_window) {
                        const double dt_t = t - mean_t;
                        sxx += dt_t * dt_t;
                        sxy += dt_t * (y - mean_y);
                    }
                    if (sxx > 1e-9) {
                        control_rate_c_per_s = std::clamp(sxy / sxx, -0.20, 0.20);
                    }
                } else if (window_span > 1e-9) {
                    // Finite-difference fallback over the endpoints of whatever
                    // window we have so far (a 2-point regression reduces to
                    // exactly this) until the smoothing path above is available.
                    control_rate_c_per_s = std::clamp(
                        (rate_window.back().second - rate_window.front().second) / window_span,
                        -0.20, 0.20);
                }


                const double prediction_tau_s = std::max(0.0, config_.pid_prediction_tau_s);
                const double configured_horizon_s = std::max(0.0, config_.pid_prediction_horizon_s);
                const double prediction_horizon_s = configured_horizon_s > 0.0 ? configured_horizon_s : std::max(0.0, current_params.interval_time_s);
                double predicted_control_temp = control_temp_filtered;
                if (prediction_tau_s > 0.0 && prediction_horizon_s > 0.0) {
                    const double response_fraction = 1.0 - std::exp(-prediction_horizon_s / prediction_tau_s);
                    predicted_control_temp = control_temp_filtered + prediction_tau_s * control_rate_c_per_s * response_fraction;
                }

                // Use NaN (not 0.0) as the "not connected" sentinel so that the
                // GUI's isnan() check works correctly even if a probe happens
                // to read exactly 0 °C (possible when min_plausible_temp_c < 0).
                const double out_t1 = t1 ? *t1 : std::numeric_limits<double>::quiet_NaN();
                const double out_t2 = t2 ? *t2 : std::numeric_limits<double>::quiet_NaN();
                const std::string s_t1 = t1 ? (fmt_double(*t1, 2) + " C") : std::string("N/C");
                const std::string s_t2 = t2 ? (fmt_double(*t2, 2) + " C") : std::string("N/C");
                log_console_quiet("TEMP  T1=" + s_t1 + "  T2=" + s_t2 + "  CTRL=" + fmt_double(avg, 2) + " C [" + active_channel + "]" +
                            (config_.pid_enabled ? ("  PRED=" + fmt_double(predicted_control_temp, 2) +
                                " C  RATE=" + fmt_double(control_rate_c_per_s, 3) +
                                " C/s  THERM=" + fmt_double(prediction_tau_s, 1) +
                                "s  HORIZ=" + fmt_double(prediction_horizon_s, 1) + "s") : std::string()));
                if (used_fallback && !temp_fallback_active) {
                    const std::string msg = "Temperature channel fallback active: " + active_channel;
                    log_console("[WARN] " + msg);
                    logger_.log_event("WARNING: " + msg);
                    temp_fallback_active = true;
                } else if (!used_fallback && temp_fallback_active) {
                    logger_.log_event("Temperature primary channel restored");
                    temp_fallback_active = false;
                }
                if (callbacks_.temperature) callbacks_.temperature(out_t1, out_t2);
                if (callbacks_.avg_temp) callbacks_.avg_temp(avg);
                logger_.log_temperature(avg, t1, t2);

                if (avg >= config_.cutoff_temp) {
                    const int min_spacing_ms = std::max(0, config_.cutoff_confirm_min_spacing_ms);
                    const bool spaced = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_over_cutoff_time).count() >= min_spacing_ms;
                    if (over_cutoff_samples == 0 || spaced) {
                        ++over_cutoff_samples;
                        last_over_cutoff_time = now;
                    } else {
                        log_console("[WARN] Over-cutoff duplicate ignored because it arrived within " + std::to_string(min_spacing_ms) + " ms of the previous over-limit sample.");
                        logger_.log_event("WARNING: over-cutoff duplicate ignored due to minimum confirmation spacing");
                    }
                    const int needed = std::max(1, config_.cutoff_confirm_samples);
                    if (over_cutoff_samples >= needed) {
                        const std::string msg = "SAFETY CUTOFF at " + fmt_double(avg, 2) + "°C";
                        log_console("[SAFETY] Cutoff at " + fmt_double(avg, 2) + "°C after " + std::to_string(over_cutoff_samples) + " consecutive over-limit samples");
                        logger_.log_error(msg);
                        if (callbacks_.cutoff) callbacks_.cutoff(avg);
                        send_stop();
                        logger_.end_session();
                        close_ultrasound_serial();
                        if (callbacks_.done) callbacks_.done(2);
                        return 2;
                    }
                    log_console("[WARN] Temperature above cutoff once (" + fmt_double(avg, 2) + "°C). Waiting for confirmation sample before cutoff.");
                    logger_.log_event("WARNING: unconfirmed over-cutoff temperature " + fmt_double(avg, 2) + " C");
                } else {
                    over_cutoff_samples = 0;
                }

                if (config_.length_mode == LengthMode::HoldAfterTarget && !target_hold_started) {
                    if (std::abs(avg - config_.pid_setpoint) <= target_tolerance) {
                        target_hold_started = true;
                        target_hold_start = now;
                        logger_.set_phase("TARGET_HOLD");
                        log_console("Target reached: " + fmt_double(avg, 2) + "°C is within ±" +
                                    fmt_double(target_tolerance, 2) + "°C of setpoint " + fmt_double(config_.pid_setpoint, 2) +
                                    "°C. Hold timer started for " + fmt_double(hold_after_target_s, 1) + "s.");
                        logger_.log_event("Target-hold timer started");
                        if (callbacks_.cycle) callbacks_.cycle("TARGET HOLD");
                    } else {
                        // set_phase is idempotent on the string — re-asserting
                        // WAIT_TARGET every sample is cheap and keeps the row
                        // labelled correctly even if a prior code path
                        // overwrote the phase column.
                        logger_.set_phase("WAIT_TARGET");
                        if (callbacks_.cycle) callbacks_.cycle("WAIT TARGET");
                    }
                }

                if (config_.pid_enabled) {
                    std::optional<double> setpoint;
                    if (phase == CyclePhase::Heating || config_.length_mode == LengthMode::HoldAfterTarget) {
                        setpoint = config_.pid_setpoint;
                    } else if (phase == CyclePhase::Cooling && config_.cooling_mode == CoolingMode::Low) {
                        setpoint = config_.cooling_hold_temp;
                    }
                    if (setpoint) {
                        const double control_temp_for_pid = prediction_tau_s > 0.0 ? predicted_control_temp : control_temp_filtered;
                        const double demand = pid.compute(*setpoint, control_temp_for_pid);
                        // Setpoint and demand are recorded as per-sample CSV
                        // columns (setpoint_C, pid_demand_pct); the verbose
                        // line stays out of the CSV to keep the file small.
                        logger_.set_setpoint(*setpoint);
                        logger_.set_pid_demand(demand);
                        log_console_quiet("PID   set=" + fmt_double(*setpoint, 2) +
                                    " C  control=" + fmt_double(control_temp_for_pid, 2) +
                                    " C  demand=" + fmt_double(demand * 100.0, 1) +
                                    "%  tau-model");
                        current_params = apply_pid_to_params(current_params, demand, initial_params,
                                                             config_.pid_amplitude, config_.pid_duration,
                                                             config_.pid_duty, config_.pid_interval);
                    }
                }
            } else {
                over_cutoff_samples = 0;
                ++invalid_temp_samples;
                log_console_quiet("TEMP  T1=N/C  T2=N/C");
                if (!config_.simulate_temp && invalid_temp_samples >= 3) {
                    const std::string msg = "Temperature sensor returned no valid data for 3 consecutive samples. Check COM port, probe connection, and HH806AU power.";
                    // PID and the operator-selected fail-closed mode both
                    // abort: the run cannot continue safely without a working
                    // temperature reading. Otherwise we degrade to monitoring-
                    // off and let the ultrasound continue.
                    if (config_.pid_enabled || config_.temperature_required) {
                        logger_.log_error(msg);
                        throw std::runtime_error(msg);
                    }
                    log_console("[WARN] " + msg + " Temperature monitoring and software cutoff are disabled for the rest of this run; ultrasound continues without PID. (Set temperature_required=true to abort instead.)");
                    logger_.log_event("WARNING: " + msg + " Monitoring disabled for rest of run.");
                    config_.temperature_enabled = false;
                }
            }
        }

        if (static_cast<int>(elapsed * 2.0) != static_cast<int>((elapsed - 0.05) * 2.0)) {
            log_console_quiet("TIME  elapsed=" + fmt_double(elapsed, 1) + "s  remaining=" + fmt_double(remaining, 1) + "s");
            if (callbacks_.time) callbacks_.time(elapsed, remaining);
        }

        if (cycling_enabled) {
            const double phase_elapsed = std::chrono::duration<double>(now - phase_start).count();
            if (phase == CyclePhase::Heating && phase_elapsed >= config_.heating_s) {
                phase = CyclePhase::Cooling;
                phase_start = now;
                pid.reset();
                logger_.set_pid_demand(std::nullopt);
                if (config_.cooling_mode == CoolingMode::Stop) {
                    send_stop();
                    sensor_->set_ultrasound_params(0.0, 0.0, 0, 1.0);
                    logger_.set_phase("COOLING");
                    logger_.set_setpoint(std::nullopt);
                    log_console("Cycle: COOLING — stopped");
                    log_console("Phase → COOLING (ultrasound stopped)");
                    if (callbacks_.cycle) callbacks_.cycle("COOLING");
                } else {
                    logger_.set_phase("COOLING_HOLD");
                    logger_.set_setpoint(config_.cooling_hold_temp);
                    log_console("Cycle: COOLING — hold");
                    log_console("Phase → COOLING (PID hold at " + fmt_double(config_.cooling_hold_temp, 1) + "°C)");
                    if (callbacks_.cycle) callbacks_.cycle("COOLING HOLD");
                }
            } else if (phase == CyclePhase::Cooling && phase_elapsed >= config_.cooling_s) {
                phase = CyclePhase::Heating;
                phase_start = now;
                pid.reset();
                logger_.set_phase("HEATING");
                logger_.set_setpoint(config_.pid_enabled ? std::optional<double>(config_.pid_setpoint) : std::nullopt);
                logger_.set_pid_demand(std::nullopt);
                log_console("Cycle: HEATING");
                log_console("Phase → HEATING");
                last_burst_valid_ = false;
                if (callbacks_.cycle) callbacks_.cycle("HEATING");
                transmit(current_params);
                last_send_time = now;
                ++cycle_count;
                logger_.log_params(current_params);
            }
        }

        const double interval_now = current_params.interval_time_s;
        if (std::chrono::duration<double>(now - last_send_time).count() >= interval_now) {
            if (!cycling_enabled || phase == CyclePhase::Heating) {
                transmit(current_params);
                last_send_time = now;
                ++cycle_count;
                logger_.log_params(current_params);
            } else if (cycling_enabled && phase == CyclePhase::Cooling && config_.cooling_mode == CoolingMode::Low) {
                transmit(current_params);
                last_send_time = now;
            }
        }

        interruptible_sleep_ms(50, stop_flag_);
    }

    // Use the latched manual-stop flag, not the live stop_flag_. The live
    // flag is briefly cleared inside send_stop() to push the STOP packet
    // through com_write(); a request_stop() landing in that window used to
    // be overwritten back to false, mis-reporting a manual stop as a clean
    // completion. The latch is monotonic and only set by request_stop /
    // force_stop, so it survives the load/clear/restore cycle.
    send_stop();
    if (manual_stop_latched_.load()) {
        log_console("Experiment stopped manually. Emergency STOP sent.");
        logger_.log_event("Manual emergency stop");
        logger_.end_session();
        close_ultrasound_serial();
        if (callbacks_.done) callbacks_.done(3);
        return 3;
    }
    if (watchdog_stop_latched_.load()) {
        // The stall watchdog had to cancel a wedged serial/UDP transfer. This
        // is an automatic recovery, not an operator action — report it as such
        // so the GUI does not raise a misleading "stopped manually" dialog.
        log_console("Experiment stopped automatically by the stall watchdog "
                    "(device stopped responding). Emergency STOP sent.");
        logger_.log_event("Watchdog auto-stop (communication stall)");
        logger_.end_session();
        close_ultrasound_serial();
        if (callbacks_.done) callbacks_.done(4);
        return 4;
    }
    log_console("Experiment complete. STOP sent.");
    logger_.log_event("Experiment complete");
    logger_.end_session();
    close_ultrasound_serial();
    if (callbacks_.done) callbacks_.done(0);
    return 0;
}

void ExperimentRunner::ensure_ultrasound_serial_open() {
    if (config_.simulate_us || ultrasound_serial_.is_open()) return;
    if (!ultrasound_serial_.open(config_.com3_port, 9600, 8, 'N', 1, 1000)) {
        throw std::runtime_error("Cannot open ultrasound serial port " + config_.com3_port);
    }
}

void ExperimentRunner::close_ultrasound_serial() {
    ultrasound_serial_.close();
}

void ExperimentRunner::com_write(const protocol::ComPacket& packet, const std::string& label) {
    if (!config_.simulate_us) {
        bool ok = false;
        std::string last_error;
        const int attempts = retry_attempts(config_);
        for (int attempt = 0; attempt < attempts && !stop_flag_; ++attempt) {
            try {
                if (config_.persistent_com3) {
                    ensure_ultrasound_serial_open();
                    ok = ultrasound_serial_.write_all(packet.data(), packet.size());
                    if (!ok) {
                        last_error = "serial write failed";
                        ultrasound_serial_.close();
                    }
                } else {
                    SerialPort ser;
                    if (!ser.open(config_.com3_port, 9600, 8, 'N', 1, 1000)) {
                        last_error = "open failed";
                        ok = false;
                    } else {
                        ok = ser.write_all(packet.data(), packet.size());
                        if (!ok) last_error = "serial write failed";
                    }
                }
            } catch (const std::exception& e) {
                ok = false;
                last_error = e.what();
                ultrasound_serial_.close();
            }
            if (ok) break;
            // If a stop was requested mid-attempt (force_stop cancels pending I/O),
            // do not waste backoff time on retries — the operator is waiting for
            // us to exit. Surface a clean abort rather than a "failed after N
            // retries" error that obscures the user's intent.
            if (stop_flag_) break;
            if (attempt + 1 < attempts) {
                const int ms = backoff_ms_for_attempt(config_, attempt);
                log_console("[WARN] " + label + " write failed on " + config_.com3_port +
                            "; retrying in " + std::to_string(ms) + " ms");
                interruptible_sleep_ms(ms, stop_flag_);
            }
        }
        if (!ok) {
            if (stop_flag_) {
                // Stopped while writing — that's expected, not an error worth
                // throwing. The caller's stop_flag check after this returns
                // will handle the exit path.
                return;
            }
            throw std::runtime_error("Failed to write ultrasound serial packet to " + config_.com3_port +
                                     (last_error.empty() ? "" : (" (" + last_error + ")")));
        }
    }
    log_console_quiet("> " + left_width(label, 5) + " " + protocol::format_hex(packet) + (config_.simulate_us ? " [SIM]" : ""));
    // COM-gap sleep also needs to be interruptible: 100 ms × 5 packets per
    // burst would otherwise add ~500 ms of latency to every emergency stop.
    interruptible_sleep_ms(static_cast<int>(COM_GAP_S * 1000.0), stop_flag_);
}

void ExperimentRunner::transmit(const ActiveParams& params) {
    try {
        const bool burst_changed = !last_burst_valid_
            || std::abs(params.amplitude - last_burst_amp_) > 1e-9
            || std::abs(params.duty_cycle - last_burst_duty_) > 1e-9
            || params.wave_shape != last_burst_shape_;

        com_write(protocol::pkt_stop(), "STOP");
        if (stop_flag_) return;
        com_write(protocol::pkt_prf(params.prf_hz), "PRF   [" + fmt_double(params.prf_hz, 0) + " Hz]");
        if (stop_flag_) return;
        com_write(protocol::pkt_cfreq(params.cfreq_hz), "CFREQ [" + fmt_double(params.cfreq_hz / 1000.0, 1) + " kHz]");
        if (stop_flag_) return;
        com_write(protocol::pkt_duration(params.duration_ms), "DUR   [" + std::to_string(params.duration_ms) + " ms]");
        if (stop_flag_) return;

        if (!config_.simulate_us) {
            if (burst_changed) {
                last_burst_ = protocol::build_udp_burst(params.amplitude, params.duty_cycle, params.wave_shape);
                // Regenerate the GUI preview vector at the same time so the
                // common steady-state case (no change) skips the 4096-sample
                // float conversion in waveform_float_array entirely.
                last_waveform_floats_ = protocol::waveform_float_array(params.amplitude, params.duty_cycle, params.wave_shape);
                last_burst_amp_ = params.amplitude;
                last_burst_duty_ = params.duty_cycle;
                last_burst_shape_ = params.wave_shape;
                last_burst_valid_ = true;
            }
            const int attempts = retry_attempts(config_);
            for (size_t i = 0; i < protocol::SAMPLE_COUNT && !stop_flag_; ++i) {
                bool sent = false;
                for (int attempt = 0; attempt < attempts && !stop_flag_; ++attempt) {
                    sent = udp_sender_.send_to(last_burst_.data() + i * 8, 8, config_.udp_host, config_.udp_port);
                    if (sent) break;
                    if (attempt + 1 < attempts) {
                        const int ms = backoff_ms_for_attempt(config_, attempt);
                        if (attempt == 0) {
                            log_console("[WARN] UDP waveform packet send failed at sample " + std::to_string(i) +
                                        "; retrying with backoff");
                        }
                        interruptible_sleep_ms(ms, stop_flag_);
                    }
                }
                if (!sent && !stop_flag_) {
                    throw std::runtime_error("Failed to send UDP waveform packet after retries at sample " + std::to_string(i));
                }
            }
        }

        if (stop_flag_) return;
        log_console_quiet("> UDP   4096 samples  amp=" + fmt_double(params.amplitude, 3) +
                    "  duty=" + fmt_double(params.duty_cycle * 100.0, 1) + "%  shape=" +
                    to_string(params.wave_shape) + (config_.simulate_us ? " [SIM]" : ""));
        if (callbacks_.params) callbacks_.params(params);
        if (callbacks_.waveform) {
            // Build the preview lazily when sim mode skipped the cache update,
            // otherwise hand the cached vector to the GUI signal (it copies).
            if (last_waveform_floats_.empty() || burst_changed) {
                last_waveform_floats_ = protocol::waveform_float_array(params.amplitude, params.duty_cycle, params.wave_shape);
            }
            callbacks_.waveform(last_waveform_floats_);
        }

        sensor_->set_ultrasound_params(params.amplitude, params.duty_cycle, params.duration_ms, params.interval_time_s);
        sensor_->set_ultrasound_active(true);
        // Update the per-sample US-active column. duration_ms=0 is the
        // protocol's "stop" command — treat that as inactive even if a burst
        // was transmitted for state reset purposes.
        logger_.set_ultrasound_active(params.duration_ms > 0);
    } catch (const std::exception& e) {
        notify_error(e.what());
        throw;
    }
}

void ExperimentRunner::emergency_stop_noexcept() {
    try {
        stop_flag_ = true;
        send_stop();
    } catch (...) {
    }
}

void ExperimentRunner::send_stop() {
    try {
        const int repeats = std::max(1, config_.emergency_stop_repeats);
        for (int i = 0; i < repeats; ++i) {
            // Temporarily clear stop_flag_ so com_write() does not bail out
            // before the hardware STOP packet is transmitted. The flag is
            // restored afterwards. A request_stop()/force_stop() landing in
            // this window will still set stop_flag_ to true, but the restore
            // will overwrite it — that's why the post-loop return-code
            // decision reads manual_stop_latched_ (monotonic) instead of
            // stop_flag_. The hardware STOP is always sent either way.
            const bool was_stop = stop_flag_.load();
            stop_flag_ = false;
            com_write(protocol::pkt_stop(), "STOP");
            stop_flag_ = was_stop;
        }
        sensor_->set_ultrasound_active(false);
        logger_.set_ultrasound_active(false);
    } catch (const std::exception& e) {
        logger_.log_error(std::string("STOP ERROR: ") + e.what());
        notify_error(std::string("STOP ERROR: ") + e.what());
    }
}

void ExperimentRunner::log_console(const std::string& msg) {
    if (callbacks_.console) callbacks_.console(msg);
    logger_.log_event(msg);
}

void ExperimentRunner::log_console_quiet(const std::string& msg) {
    if (callbacks_.console) callbacks_.console(msg);
    // Skips the CSV event row. The in-memory ring is still appended to so the
    // metadata JSON shows the recent timeline.
    logger_.log_console_only(msg);
}

void ExperimentRunner::notify_error(const std::string& msg) {
    logger_.log_error(msg);
    const std::string full = std::string("[ERROR] ") + msg;
    if (callbacks_.error) callbacks_.error(msg);
    log_console(full);
}

} // namespace sonocontrol
