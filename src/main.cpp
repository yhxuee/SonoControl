#include "config.hpp"
#include "config_io.hpp"
#include "experiment.hpp"
#include "logger.hpp"
#include "protocol.hpp"
#include "temperature.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef SONOCONTROL_DEBUG_SIM
#define SONOCONTROL_DEBUG_SIM 0
#endif

namespace sonocontrol {
namespace {

bool is_debug_sim_build() {
#if SONOCONTROL_DEBUG_SIM
    return true;
#else
    return false;
#endif
}

void print_usage(const char* exe) {
    std::cout
        << "SonoControl C++ executable\n"
        << "\nUsage:\n"
        << "  " << exe << " [options]\n"
        << "\nCore options:\n"
        << "  --amplitude X          Ultrasound amplitude 0..1, default 0.5\n"
        << "  --cfreq-khz X          Carrier frequency in kHz, default 500\n"
        << "  --cfreq-hz X           Carrier frequency in Hz\n"
        << "  --prf-hz X             PRF in Hz, default 1000\n"
        << "  --duty-pct X           Duty cycle in percent, default 50\n"
        << "  --duration-ms N        Duration in ms, default 1000\n"
        << "  --interval-s X         Interval in seconds, default 5\n"
        << "  --wave sine|square|triangle\n"
        << "  --total-min X          Run by total duration, default 60\n"
        << "  --total-s X            Run by total seconds, useful for tests\n"
        << "  --repeating N          Run by repeated intervals instead of total duration; not allowed with --pid\n"
        << "  --hold-after-target-min X  Start finish timer after setpoint is reached, then hold for X min\n"
        << "  --target-tolerance X   Target-reached tolerance in °C, default 0.3\n"
        << "  --sampling-hz X        Temperature sampling rate, default 2\n"
        << "  --cutoff-temp X        Safety cutoff temperature, default 45; active only with --pid\n"
        << "  --config FILE          Load a .config file; later CLI options override it\n"
        << "  --write-config-template FILE  Write a tau-only .config template and exit\n"
        << "\nPID options:\n"
        << "  --pid                  Enable PID control and HH806AU temperature sampling\n"
        << "  --setpoint X           PID setpoint, default 40\n"
        << "  --pid-kp X --pid-ki X --pid-kd X\n"
        << "  --pid-tau X            Thermal constant seconds for T_future=T+tau*dT/dt*(1-exp(-horizon/tau)); 0 disables prediction\n"
        << "  --pid-horizon X        Prediction horizon seconds; 0 uses current hardware interval\n"
        << "  --auto-save-dir DIR    Completed experiment artifact folder; also supported in .config\n"
        << "  --pid-duration         PID adjusts duration\n"
        << "  --pid-duty             PID adjusts duty cycle\n"
        << "  --pid-interval         PID adjusts interval\n"
        << "  --no-pid-amplitude     Disable PID amplitude adjustment\n"
        << "\nCycle options:\n"
        << "  --cycling              Enable heating/cooling cycles\n"
        << "  --heating-s X          Heating phase seconds, default 60\n"
        << "  --cooling-s X          Cooling phase seconds, default 30\n"
        << "  --cooling-mode stop|low\n"
        << "  --cooling-hold-temp X  Low-power cooling hold temperature\n"
        << "\nHardware options:\n"
        << "  --com3 PORT            Ultrasound FPGA serial port, default COM3\n"
        << "  --com11 PORT           HH806AU serial port, default COM5\n"
        << "  --udp-host HOST        UDP target, default 192.168.0.2\n"
        << "  --udp-port PORT        UDP target port, default 5000\n"
        << "  --temp-channel t1|t2|avg\n"
        << "\nDebug-build-only options:\n"
        << "  --sim-temp             Use simulated thermometer\n"
        << "  --sim-us               Simulate serial/UDP ultrasound transport\n"
        << "  --real-hardware        Disable simulations in a Debug build\n"
        << "\nOther:\n"
        << "  --protocol-check       Print known packet hex and exit\n"
        << "  --help                 Show this help\n";
}

double parse_double(const std::string& s, const std::string& name) {
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') throw std::invalid_argument("Invalid number for " + name + ": " + s);
    return v;
}

int parse_int(const std::string& s, const std::string& name) {
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') throw std::invalid_argument("Invalid integer for " + name + ": " + s);
    return static_cast<int>(v);
}

std::string need_value(const std::vector<std::string>& args, size_t& i, const std::string& opt) {
    if (i + 1 >= args.size()) throw std::invalid_argument("Missing value for " + opt);
    return args[++i];
}

TempChannel parse_temp_channel(const std::string& s) {
    std::string v = s;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v == "t1" || v == "0") return TempChannel::T1;
    if (v == "t2" || v == "1") return TempChannel::T2;
    if (v == "avg" || v == "average" || v == "2") return TempChannel::Average;
    throw std::invalid_argument("Invalid temp channel: " + s);
}

void validate_config(Config& c) {
    c.amplitude = std::clamp(c.amplitude, 0.0, 1.0);
    c.duty_cycle = std::clamp(c.duty_cycle, 0.0, 1.0);
    c.duration_ms = std::max(0, c.duration_ms);
    // Interval floor — see sonocontrol::kMinIntervalTimeS in config.hpp.
    // The GUI enforces the same floor at the input widget; this guard
    // catches CLI flags (e.g. --interval-s 0.01) and config-file values
    // that would otherwise bypass it.
    {
        const double requested = c.interval_time_s;
        const double duration_floor = static_cast<double>(c.duration_ms) / 1000.0;
        const double clamped = std::max({requested, duration_floor, kMinIntervalTimeS});
        if (clamped > requested) {
            std::cerr << "[WARN] interval_time_s=" << requested
                      << " s is below the realistic per-cycle floor ("
                      << kMinIntervalTimeS << " s and >= duration); raised to "
                      << clamped << " s.\n";
        }
        c.interval_time_s = clamped;
    }
    c.sampling_rate_hz = std::max(0.1, c.sampling_rate_hz);
    c.total_duration_mins = std::max(0.0, c.total_duration_mins);
    c.repeating = std::max(1, c.repeating);
    c.hold_after_target_mins = std::max(0.0, c.hold_after_target_mins);
    c.target_tolerance_c = std::clamp(c.target_tolerance_c, 0.05, 5.0);
    c.use_total_duration = (c.length_mode != LengthMode::RepeatingCycles);
    if (c.pid_enabled && c.length_mode == LengthMode::RepeatingCycles) {
        throw std::invalid_argument("--repeating cannot be used with --pid; use --total-min/--total-s or --hold-after-target-min");
    }
    if (c.length_mode == LengthMode::HoldAfterTarget) {
        c.temperature_enabled = true;
        c.use_cycling = false;
    }
    if (c.use_cycling && c.length_mode != LengthMode::TotalDuration) c.use_cycling = false;
    c.udp_port = std::max<uint16_t>(1, c.udp_port);
    c.pid_prediction_tau_s = std::clamp(c.pid_prediction_tau_s, 0.0, 3600.0);
    c.pid_prediction_horizon_s = std::clamp(c.pid_prediction_horizon_s, 0.0, 360.0);
}

Config parse_args(int argc, char** argv, bool& protocol_check, bool& help, std::string& write_template_path) {
    Config c;
    c.simulate_temp = is_debug_sim_build();
    c.simulate_us = is_debug_sim_build();

    std::vector<std::string> args(argv + 1, argv + argc);
    for (size_t i = 0; i < args.size(); ++i) {
        const auto& a = args[i];
        if (a == "--help" || a == "-h") { help = true; continue; }
        if (a == "--protocol-check") { protocol_check = true; continue; }
        if (a == "--write-config-template") { write_template_path = need_value(args, i, a); continue; }
        if (a == "--config") { c = load_config_file(need_value(args, i, a)); c.config_source_type = "cli-loaded-config"; continue; }
        if (a == "--amplitude") c.amplitude = parse_double(need_value(args, i, a), a);
        else if (a == "--cfreq-khz") c.cfreq_hz = parse_double(need_value(args, i, a), a) * 1000.0;
        else if (a == "--cfreq-hz") c.cfreq_hz = parse_double(need_value(args, i, a), a);
        else if (a == "--prf-hz") c.prf_hz = parse_double(need_value(args, i, a), a);
        else if (a == "--duty-pct") c.duty_cycle = parse_double(need_value(args, i, a), a) / 100.0;
        else if (a == "--duration-ms") c.duration_ms = parse_int(need_value(args, i, a), a);
        else if (a == "--interval-s") c.interval_time_s = parse_double(need_value(args, i, a), a);
        else if (a == "--wave") c.wave_shape = wave_shape_from_string(need_value(args, i, a));
        else if (a == "--total-min") { c.length_mode = LengthMode::TotalDuration; c.use_total_duration = true; c.total_duration_mins = parse_double(need_value(args, i, a), a); }
        else if (a == "--total-s") { c.length_mode = LengthMode::TotalDuration; c.use_total_duration = true; c.total_duration_mins = parse_double(need_value(args, i, a), a) / 60.0; }
        else if (a == "--repeating") { c.length_mode = LengthMode::RepeatingCycles; c.use_total_duration = false; c.repeating = parse_int(need_value(args, i, a), a); }
        else if (a == "--hold-after-target-min") { c.length_mode = LengthMode::HoldAfterTarget; c.use_total_duration = true; c.hold_after_target_mins = parse_double(need_value(args, i, a), a); c.temperature_enabled = true; }
        else if (a == "--target-tolerance") c.target_tolerance_c = parse_double(need_value(args, i, a), a);
        else if (a == "--sampling-hz") c.sampling_rate_hz = parse_double(need_value(args, i, a), a);
        else if (a == "--cutoff-temp") c.cutoff_temp = parse_double(need_value(args, i, a), a);
        else if (a == "--pid") { c.pid_enabled = true; c.temperature_enabled = true; }
        else if (a == "--setpoint") c.pid_setpoint = parse_double(need_value(args, i, a), a);
        else if (a == "--pid-kp") c.pid_kp = parse_double(need_value(args, i, a), a);
        else if (a == "--pid-ki") c.pid_ki = parse_double(need_value(args, i, a), a);
        else if (a == "--pid-kd") c.pid_kd = parse_double(need_value(args, i, a), a);
        else if (a == "--pid-tau") c.pid_prediction_tau_s = parse_double(need_value(args, i, a), a);
        else if (a == "--pid-horizon") c.pid_prediction_horizon_s = parse_double(need_value(args, i, a), a);
        else if (a == "--auto-save-dir") c.auto_save_dir = need_value(args, i, a);
        else if (a == "--pid-duration") c.pid_duration = true;
        else if (a == "--pid-duty") c.pid_duty = true;
        else if (a == "--pid-interval") c.pid_interval = true;
        else if (a == "--no-pid-amplitude") c.pid_amplitude = false;
        else if (a == "--cycling") c.use_cycling = true;
        else if (a == "--heating-s") c.heating_s = parse_double(need_value(args, i, a), a);
        else if (a == "--cooling-s") c.cooling_s = parse_double(need_value(args, i, a), a);
        else if (a == "--cooling-mode") c.cooling_mode = cooling_mode_from_string(need_value(args, i, a));
        else if (a == "--cooling-hold-temp") c.cooling_hold_temp = parse_double(need_value(args, i, a), a);
        else if (a == "--com3") c.com3_port = need_value(args, i, a);
        else if (a == "--com11") c.com11_port = need_value(args, i, a);
        else if (a == "--udp-host") c.udp_host = need_value(args, i, a);
        else if (a == "--udp-port") {
            const int v = parse_int(need_value(args, i, a), a);
            if (v < 1 || v > 65535) throw std::invalid_argument("--udp-port must be between 1 and 65535: " + std::to_string(v));
            c.udp_port = static_cast<uint16_t>(v);
        }
        else if (a == "--temp-channel") c.temp_channel = parse_temp_channel(need_value(args, i, a));
        else if (a == "--real-hardware") { c.simulate_temp = false; c.simulate_us = false; }
        else if (a == "--sim-temp") {
            if (is_debug_sim_build()) c.simulate_temp = true;
            else std::cerr << "[WARN] --sim-temp ignored: this is not a Debug build.\n";
        }
        else if (a == "--sim-us") {
            if (is_debug_sim_build()) c.simulate_us = true;
            else std::cerr << "[WARN] --sim-us ignored: this is not a Debug build.\n";
        }
        else {
            throw std::invalid_argument("Unknown option: " + a);
        }
    }

    if (!is_debug_sim_build()) {
        c.simulate_temp = false;
        c.simulate_us = false;
    }
    c.temperature_enabled = c.temperature_enabled || c.pid_enabled || c.length_mode == LengthMode::HoldAfterTarget;
    if (!c.temperature_enabled) c.simulate_temp = false;
    validate_config(c);
    return c;
}

void run_protocol_check() {
    const auto stop = protocol::pkt_stop();
    const auto prf = protocol::pkt_prf(1000.0);
    const auto cfreq = protocol::pkt_cfreq(500000.0);
    const auto dur = protocol::pkt_duration(1000);
    const auto sample0 = protocol::udp_sample(0, protocol::build_amplitude_table(0.5, 0.5, WaveShape::Sine)[0]);
    const auto sample_mid = protocol::udp_sample(1024, protocol::build_amplitude_table(0.5, 0.5, WaveShape::Sine)[1024]);
    std::cout << "STOP  " << protocol::format_hex(stop) << '\n';
    std::cout << "PRF   " << protocol::format_hex(prf) << '\n';
    std::cout << "CFREQ " << protocol::format_hex(cfreq) << '\n';
    std::cout << "DUR   " << protocol::format_hex(dur) << '\n';
    std::cout << "UDP0  " << protocol::format_hex(sample0.data(), sample0.size()) << '\n';
    std::cout << "UDP1024 " << protocol::format_hex(sample_mid.data(), sample_mid.size()) << '\n';
}

} // namespace
} // namespace sonocontrol

int main(int argc, char** argv) {
    try {
        bool protocol_check = false;
        bool help = false;
        std::string write_template_path;
        auto config = sonocontrol::parse_args(argc, argv, protocol_check, help, write_template_path);
        if (help) {
            sonocontrol::print_usage(argv[0]);
            return 0;
        }
        if (!write_template_path.empty()) {
            sonocontrol::write_config_template(write_template_path);
            std::cout << "Wrote config template: " << write_template_path << std::endl;
            return 0;
        }
        if (protocol_check) {
            sonocontrol::run_protocol_check();
            return 0;
        }

        std::unique_ptr<sonocontrol::ITemperatureSensor> sensor;
        if (!config.temperature_enabled) {
            sensor = std::make_unique<sonocontrol::NullTemperatureSensor>();
        } else if (config.simulate_temp) {
            sensor = std::make_unique<sonocontrol::TemperatureSimulator>();
        } else {
            sensor = std::make_unique<sonocontrol::HH806AUSensor>(config.com11_port, config.min_plausible_temp_c, config.max_plausible_temp_c, config.max_temp_rate_c_per_s);
        }

        sonocontrol::ExperimentLogger logger;
        sonocontrol::ExperimentRunner runner(
            config,
            std::move(sensor),
            logger,
            [](const std::string& msg) { std::cout << '[' << sonocontrol::now_hms() << "] " << msg << std::endl; });

        std::cout << "SonoControl C++ "
                  << (sonocontrol::is_debug_sim_build() ? "Debug(sim enabled)" : "Release(sim disabled)")
                  << std::endl;
        return runner.run();
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }
}
