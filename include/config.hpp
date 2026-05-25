#pragma once

#include <cstdint>
#include <string>

namespace sonocontrol {

// Wall-clock floor for `interval_time_s`. Each transmit cycle sends a serial
// CFREQ/PRF/DUR triple (each followed by the hardware-required 100 ms COM
// gap) plus the 4096-datagram UDP waveform burst; the end-to-end time is
// ~0.3–0.5 s on a healthy local link. A smaller interval cannot make the
// next cycle start any sooner — it just causes cycles to overrun their
// scheduled slot and stretches the run beyond the configured total. Both
// the GUI's validate_config and the CLI's validate_config clamp to this
// value so a config file, CLI flag, and GUI spinbox cannot disagree.
inline constexpr double kMinIntervalTimeS = 0.2;

enum class WaveShape { Sine, Square, Triangle };
enum class TempChannel { T1 = 0, T2 = 1, Average = 2 };
enum class CoolingMode { Stop, Low };
enum class LengthMode { TotalDuration, RepeatingCycles, HoldAfterTarget };

struct Config {
    double amplitude = 0.5;          // 0..1
    double cfreq_hz = 500000.0;      // original UI: kHz input * 1000
    double prf_hz = 1000.0;
    double duty_cycle = 0.50;        // fraction, not percent
    WaveShape wave_shape = WaveShape::Sine;
    int duration_ms = 1000;
    double interval_time_s = 5.0;

    // Experiment length mode. TotalDuration preserves the original behavior.
    // RepeatingCycles is disabled automatically when PID is enabled.
    // HoldAfterTarget starts the finish timer after the selected temperature channel first
    // reaches pid_setpoint ± target_tolerance_c; it requires temperature monitoring.
    LengthMode length_mode = LengthMode::TotalDuration;
    bool use_total_duration = true;  // legacy compatibility mirror: true except RepeatingCycles
    double total_duration_mins = 60.0;
    int repeating = 720;
    double hold_after_target_mins = 10.0;
    double target_tolerance_c = 0.3;

    double sampling_rate_hz = 2.0;
    double cutoff_temp = 45.0;
    int cutoff_confirm_samples = 2;  // require consecutive over-cutoff samples to reject single serial-frame spikes
    int cutoff_confirm_min_spacing_ms = 150;

    // HH806AU plausibility guards. Keep these configurable; do not hard-code a
    // biological temperature window in the decoder because some protocols may
    // intentionally use cold-shock or high-temperature calibration runs.
    double min_plausible_temp_c = 10.0;
    double max_plausible_temp_c = 80.0;
    double max_temp_rate_c_per_s = 15.0;
    bool temp_channel_fallback = false;  // enable only when both probes are physically installed

    // Temperature monitoring is independent from PID. When false, HH806AU is
    // not opened and temperature sampling/cutoff are skipped. PID requires
    // temperature_enabled=true and preflight must verify a valid sensor read.
    bool temperature_enabled = false;
    // Fail-closed: when true, sustained loss of the temperature sensor
    // (3 consecutive invalid samples in non-PID mode) aborts the run instead
    // of silently disabling monitoring and continuing ultrasound. PID and
    // hold-after-target already abort by construction (they require the
    // sensor); this flag extends the same behavior to monitoring-only runs
    // where the operator has explicitly committed to a temperature limit.
    bool temperature_required = false;

    bool pid_enabled = false;
    double pid_setpoint = 40.0;
    bool pid_amplitude = true;
    bool pid_duration = false;
    bool pid_duty = false;
    bool pid_interval = false;
    double pid_kp = 0.8;
    double pid_ki = 0.05;
    double pid_kd = 0.2;
    // Thermal time constant used by the predictive model:
    // T_future = T_now + tau * dT/dt * (1 - exp(-t1/tau)).
    // 0 disables prediction and falls back to the filtered measured temperature.
    double pid_prediction_tau_s = 25.0;

    // Prediction horizon t1 in seconds. 0 = use current hardware interval.
    double pid_prediction_horizon_s = 0.0;

    // Default folder for completed experiment artifacts. Empty = ./experiments/<session>.
    std::string auto_save_dir;

    bool use_cycling = false;
    double heating_s = 60.0;
    double cooling_s = 30.0;
    CoolingMode cooling_mode = CoolingMode::Stop;
    double cooling_hold_temp = 37.0;

    std::string com3_port = "COM3";
    std::string com11_port = "COM5";
    TempChannel temp_channel = TempChannel::T1;
    std::string udp_host = "192.168.0.2";
    uint16_t udp_port = 5000;

    bool simulate_temp = false;
    bool simulate_us = false;

    // Long-run stability defaults. These do not alter the external serial/UDP
    // protocol; they only control retry/stop behavior around the same packets.
    bool persistent_com3 = true;
    bool communication_retry = true;
    int communication_retry_attempts = 3;
    int communication_retry_initial_backoff_ms = 80;
    int emergency_stop_repeats = 5;
    int watchdog_timeout_ms = 5000;

    // Configuration provenance; written to CSV/meta logs.
    std::string config_source_type = "gui-default";
    std::string config_file_path;

    // Optional in-app monitoring web server. The `enabled` flag is session-
    // only — explicitly NOT serialized into .config files, so loading a
    // shared config can never silently expose the machine on a network. Port,
    // snapshot interval, and LAN-bind flag are persisted because a setup of
    // {port=50896, interval=15 min, lan=off} is a fine recurring default.
    bool web_server_enabled = false;
    uint16_t web_server_port = 50896;
    int web_server_snapshot_interval_s = 900;  // 15 minutes; see kMinIntervalTimeS-style rationale in README
    bool web_server_lan = false;               // false = bind 127.0.0.1; true = bind all interfaces
};

struct ActiveParams {
    double amplitude = 0.0;
    double cfreq_hz = 0.0;
    double prf_hz = 0.0;
    double duty_cycle = 0.0;
    int duration_ms = 0;
    double interval_time_s = 0.0;
    WaveShape wave_shape = WaveShape::Sine;
};

const char* to_string(WaveShape shape);
WaveShape wave_shape_from_string(const std::string& s);

const char* to_string(CoolingMode mode);
CoolingMode cooling_mode_from_string(const std::string& s);

const char* to_string(LengthMode mode);
LengthMode length_mode_from_string(const std::string& s);

} // namespace sonocontrol
