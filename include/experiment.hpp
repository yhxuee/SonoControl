#pragma once

#include "config.hpp"
#include "logger.hpp"
#include "protocol.hpp"
#include "udp_socket.hpp"
#include "temperature.hpp"
#include "serial_port.hpp"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace sonocontrol {

struct ExperimentCallbacks {
    std::function<void(const std::string&)> console;
    std::function<void(double, double)> temperature;
    std::function<void(double)> avg_temp;
    std::function<void(const ActiveParams&)> params;
    std::function<void(double, double)> time;
    std::function<void(const std::string&)> cycle;
    std::function<void(const std::vector<float>&)> waveform;
    std::function<void(double)> cutoff;
    std::function<void(const std::string&)> error;
    std::function<void(int)> done;
};

class ExperimentRunner {
public:
    using ConsoleCallback = std::function<void(const std::string&)>;

    ExperimentRunner(Config config,
                     std::unique_ptr<ITemperatureSensor> sensor,
                     ExperimentLogger& logger,
                     ConsoleCallback console);

    ExperimentRunner(Config config,
                     std::unique_ptr<ITemperatureSensor> sensor,
                     ExperimentLogger& logger,
                     ExperimentCallbacks callbacks);

    void request_stop();
    void emergency_stop_noexcept();
    // Aggressive *operator-initiated* stop: sets the stop flag AND cancels any
    // pending serial/UDP I/O so a wedged WriteFile/sendto returns immediately.
    // Latches manual intent → run() reports code 3 (manual). Used by the
    // EMERGENCY STOP button and its 2 s graceful→force escalation.
    void force_stop();
    // Same I/O-cancelling mechanism, but latches a *watchdog* stop instead of a
    // manual one so run() reports code 4 (automatic comms-stall stop). Called by
    // the GUI stall watchdog when the worker wedges in a blocking send/write —
    // keeps an automatic recovery from being mis-reported as an operator stop.
    void watchdog_stop();
    int run();

private:
    UdpSender udp_sender_;
    SerialPort ultrasound_serial_;
    // Cached UDP waveform burst (4096 × 8 bytes) and the matching GUI preview
    // (4096 floats). Invariant: protocol::build_udp_burst() and
    // protocol::waveform_float_array() are pure functions of exactly three
    // inputs — amplitude, duty cycle, and wave shape. Carrier frequency
    // (cfreq_hz), PRF, and burst duration are transmitted as *separate*
    // COM serial packets (CFREQ/PRF/DUR) and are not encoded in the UDP
    // amplitude table; PID-controlled changes to duration_ms and
    // interval_time_s likewise do not enter the table. Therefore the cache
    // key (last_burst_amp_, last_burst_duty_, last_burst_shape_) is complete
    // and correct for the actual table inputs that can change at run time.
    //
    // If a future change adds another argument to build_udp_burst or
    // waveform_float_array, that argument MUST also become part of the cache
    // key — extend the burst_changed predicate in transmit() at the same
    // time, otherwise the FPGA will receive a stale envelope.
    std::vector<uint8_t> last_burst_;
    std::vector<float> last_waveform_floats_;
    double last_burst_amp_ = -1.0;
    double last_burst_duty_ = -1.0;
    WaveShape last_burst_shape_ = WaveShape::Sine;
    bool last_burst_valid_ = false;
    enum class CyclePhase { Heating, Cooling };

    void transmit(const ActiveParams& params);
    void send_stop();
    void close_ultrasound_serial();
    void ensure_ultrasound_serial_open();
    void com_write(const protocol::ComPacket& packet, const std::string& label);
    // log_console writes the message to the CSV event column AND to the GUI
    // console. log_console_quiet skips the CSV row — use it for chatty,
    // duplicative output (per-packet hex dumps, periodic TEMP/TIME/PID
    // status) so the CSV stays small on long runs.
    void log_console(const std::string& msg);
    void log_console_quiet(const std::string& msg);
    void notify_error(const std::string& msg);
    static ActiveParams initial_params_from_config(const Config& c);

    Config config_;
    std::unique_ptr<ITemperatureSensor> sensor_;
    ExperimentLogger& logger_;
    ExperimentCallbacks callbacks_;
    std::atomic<bool> stop_flag_{false};
    // Latched manual-stop flag. Set the moment request_stop() / force_stop()
    // is called from the GUI; never cleared while the run is in progress.
    // The end-of-run code-selection logic reads this — not stop_flag_ — so a
    // stop request that lands during the load/clear/restore window inside
    // send_stop() still reports return code 3 (manual) instead of being
    // overwritten back to 0 (complete). See the comment in send_stop().
    std::atomic<bool> manual_stop_latched_{false};
    // Latched watchdog-stop flag, set only by watchdog_stop(). Distinct from
    // manual_stop_latched_ so the end-of-run logic can report an automatic
    // comms-stall recovery (code 4) separately from an operator stop (code 3).
    // Manual intent takes precedence if both are somehow latched.
    std::atomic<bool> watchdog_stop_latched_{false};
};

} // namespace sonocontrol
