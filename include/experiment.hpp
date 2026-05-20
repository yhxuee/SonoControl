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
    // Aggressive stop: sets the stop flag AND cancels any pending serial/UDP
    // I/O on the worker thread so a wedged WriteFile/sendto returns immediately.
    // Safe to call from any thread, including from a GUI watchdog timer.
    void force_stop();
    int run();

private:
    UdpSender udp_sender_;
    SerialPort ultrasound_serial_;
    std::vector<uint8_t> last_burst_;
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
    void log_console(const std::string& msg);
    void notify_error(const std::string& msg);
    static ActiveParams initial_params_from_config(const Config& c);

    Config config_;
    std::unique_ptr<ITemperatureSensor> sensor_;
    ExperimentLogger& logger_;
    ExperimentCallbacks callbacks_;
    std::atomic<bool> stop_flag_{false};
};

} // namespace sonocontrol
