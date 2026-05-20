#include "config_io.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace sonocontrol {
namespace {

std::string trim(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool parse_bool(const std::string& raw, const std::string& key) {
    const auto v = lower(trim(raw));
    if (v == "true" || v == "yes" || v == "on" || v == "1") return true;
    if (v == "false" || v == "no" || v == "off" || v == "0") return false;
    throw std::invalid_argument("Invalid boolean for " + key + ": " + raw);
}

double parse_double(const std::string& raw, const std::string& key) {
    char* end = nullptr;
    const std::string s = trim(raw);
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') throw std::invalid_argument("Invalid number for " + key + ": " + raw);
    return v;
}

int parse_int(const std::string& raw, const std::string& key) {
    char* end = nullptr;
    const std::string s = trim(raw);
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') throw std::invalid_argument("Invalid integer for " + key + ": " + raw);
    return static_cast<int>(v);
}

TempChannel temp_channel_from_string(const std::string& raw) {
    const auto v = lower(trim(raw));
    if (v == "t1" || v == "0") return TempChannel::T1;
    if (v == "t2" || v == "1") return TempChannel::T2;
    if (v == "avg" || v == "average" || v == "2") return TempChannel::Average;
    throw std::invalid_argument("Invalid temp_channel: " + raw);
}

std::string temp_channel_to_string(TempChannel ch) {
    if (ch == TempChannel::T1) return "t1";
    if (ch == TempChannel::T2) return "t2";
    return "avg";
}

void parse_kv(Config& c, const std::string& key_raw, const std::string& value) {
    const auto key = lower(trim(key_raw));
    if (key == "amplitude") c.amplitude = parse_double(value, key);
    else if (key == "cfreq_hz") c.cfreq_hz = parse_double(value, key);
    else if (key == "cfreq_khz") c.cfreq_hz = parse_double(value, key) * 1000.0;
    else if (key == "prf_hz") c.prf_hz = parse_double(value, key);
    else if (key == "duty_cycle") c.duty_cycle = parse_double(value, key);
    else if (key == "duty_pct") c.duty_cycle = parse_double(value, key) / 100.0;
    else if (key == "wave_shape") c.wave_shape = wave_shape_from_string(trim(value));
    else if (key == "duration_ms") c.duration_ms = parse_int(value, key);
    else if (key == "interval_time_s") c.interval_time_s = parse_double(value, key);
    else if (key == "length_mode") c.length_mode = length_mode_from_string(trim(value));
    else if (key == "use_total_duration") c.use_total_duration = parse_bool(value, key);
    else if (key == "total_duration_mins") c.total_duration_mins = parse_double(value, key);
    else if (key == "repeating") c.repeating = parse_int(value, key);
    else if (key == "hold_after_target_mins") c.hold_after_target_mins = parse_double(value, key);
    else if (key == "target_tolerance_c") c.target_tolerance_c = parse_double(value, key);
    else if (key == "sampling_rate_hz") c.sampling_rate_hz = parse_double(value, key);
    else if (key == "cutoff_temp") c.cutoff_temp = parse_double(value, key);
    else if (key == "cutoff_confirm_samples") c.cutoff_confirm_samples = parse_int(value, key);
    else if (key == "cutoff_confirm_min_spacing_ms") c.cutoff_confirm_min_spacing_ms = parse_int(value, key);
    else if (key == "min_plausible_temp_c") c.min_plausible_temp_c = parse_double(value, key);
    else if (key == "max_plausible_temp_c") c.max_plausible_temp_c = parse_double(value, key);
    else if (key == "max_temp_rate_c_per_s") c.max_temp_rate_c_per_s = parse_double(value, key);
    else if (key == "temp_channel_fallback") c.temp_channel_fallback = parse_bool(value, key);
    else if (key == "temperature_enabled") c.temperature_enabled = parse_bool(value, key);
    else if (key == "pid_enabled") c.pid_enabled = parse_bool(value, key);
    else if (key == "pid_setpoint") c.pid_setpoint = parse_double(value, key);
    else if (key == "pid_amplitude") c.pid_amplitude = parse_bool(value, key);
    else if (key == "pid_duration") c.pid_duration = parse_bool(value, key);
    else if (key == "pid_duty") c.pid_duty = parse_bool(value, key);
    else if (key == "pid_interval") c.pid_interval = parse_bool(value, key);
    else if (key == "pid_kp") c.pid_kp = parse_double(value, key);
    else if (key == "pid_ki") c.pid_ki = parse_double(value, key);
    else if (key == "pid_kd") c.pid_kd = parse_double(value, key);
    else if (key == "pid_prediction_tau_s") c.pid_prediction_tau_s = parse_double(value, key);
    else if (key == "pid_prediction_horizon_s") c.pid_prediction_horizon_s = parse_double(value, key);
    else if (key == "auto_save_dir") c.auto_save_dir = trim(value);
    else if (key == "pid_dynamic_tau_enabled" || key == "pid_tau_rate_low_c_per_s" ||
             key == "pid_tau_cap_low_s" || key == "pid_tau_rate_mid_c_per_s" ||
             key == "pid_tau_cap_mid_s" || key == "pid_tau_rate_high_c_per_s" ||
             key == "pid_tau_cap_high_s") {
        // Obsolete dynamic-tau keys from older tau-only builds are accepted but ignored.
    }
    else if (key == "use_cycling") c.use_cycling = parse_bool(value, key);
    else if (key == "heating_s") c.heating_s = parse_double(value, key);
    else if (key == "cooling_s") c.cooling_s = parse_double(value, key);
    else if (key == "cooling_mode") c.cooling_mode = cooling_mode_from_string(trim(value));
    else if (key == "cooling_hold_temp") c.cooling_hold_temp = parse_double(value, key);
    else if (key == "com3_port") c.com3_port = trim(value);
    else if (key == "com11_port") c.com11_port = trim(value);
    else if (key == "temp_channel") c.temp_channel = temp_channel_from_string(value);
    else if (key == "udp_host") c.udp_host = trim(value);
    else if (key == "udp_port") c.udp_port = static_cast<uint16_t>(parse_int(value, key));
    else if (key == "simulate_temp") c.simulate_temp = parse_bool(value, key);
    else if (key == "simulate_us") c.simulate_us = parse_bool(value, key);
    else if (key == "persistent_com3") c.persistent_com3 = parse_bool(value, key);
    else if (key == "communication_retry") c.communication_retry = parse_bool(value, key);
    else if (key == "communication_retry_attempts") c.communication_retry_attempts = parse_int(value, key);
    else if (key == "communication_retry_initial_backoff_ms") c.communication_retry_initial_backoff_ms = parse_int(value, key);
    else if (key == "emergency_stop_repeats") c.emergency_stop_repeats = parse_int(value, key);
    else if (key == "watchdog_timeout_ms") c.watchdog_timeout_ms = parse_int(value, key);
    else if (key == "config_source_type" || key == "config_file_path") {
        // Runtime provenance is set by the loader/UI, not trusted from file content.
    } else {
        throw std::invalid_argument("Unknown configuration key: " + key_raw);
    }
}

void write_bool(std::ostream& os, const char* key, bool v) { os << key << " = " << (v ? "true" : "false") << '\n'; }
void write_num(std::ostream& os, const char* key, double v) { os << key << " = " << std::setprecision(12) << v << '\n'; }
void write_int(std::ostream& os, const char* key, int v) { os << key << " = " << v << '\n'; }
void write_str(std::ostream& os, const char* key, const std::string& v) { os << key << " = " << v << '\n'; }

} // namespace

Config load_config_file(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open configuration file: " + path.string());
    Config c;
    std::string line;
    int line_no = 0;
    while (std::getline(f, line)) {
        ++line_no;
        auto s = trim(line);
        if (s.empty() || s[0] == '#' || s[0] == ';') continue;
        if (s.front() == '[' && s.back() == ']') continue;
        auto pos = s.find('=');
        if (pos == std::string::npos) {
            throw std::invalid_argument("Invalid config line " + std::to_string(line_no) + ": " + line);
        }
        parse_kv(c, s.substr(0, pos), s.substr(pos + 1));
    }
    c.config_source_type = "loaded-config";
    c.config_file_path = path.string();
    return c;
}

std::string config_to_text(const Config& c, bool include_comments) {
    std::ostringstream os;
    if (include_comments) {
        os << "# SonoControl configuration file\n";
        os << "# Format: key = value. Lines beginning with # or ; are ignored.\n";
        os << "# This template intentionally contains no max_demand, hold-demand, or dynamic-tau controls.\n\n";
    }
    os << "[ultrasound]\n";
    write_num(os, "amplitude", c.amplitude);
    write_num(os, "cfreq_hz", c.cfreq_hz);
    write_num(os, "prf_hz", c.prf_hz);
    write_num(os, "duty_cycle", c.duty_cycle);
    write_str(os, "wave_shape", to_string(c.wave_shape));
    write_int(os, "duration_ms", c.duration_ms);
    write_num(os, "interval_time_s", c.interval_time_s);
    os << '\n';

    os << "[experiment]\n";
    write_str(os, "length_mode", to_string(c.length_mode));
    write_bool(os, "use_total_duration", c.use_total_duration);
    write_num(os, "total_duration_mins", c.total_duration_mins);
    write_int(os, "repeating", c.repeating);
    write_num(os, "hold_after_target_mins", c.hold_after_target_mins);
    write_num(os, "target_tolerance_c", c.target_tolerance_c);
    write_num(os, "sampling_rate_hz", c.sampling_rate_hz);
    os << '\n';

    os << "[temperature_safety]\n";
    write_num(os, "cutoff_temp", c.cutoff_temp);
    write_int(os, "cutoff_confirm_samples", c.cutoff_confirm_samples);
    write_int(os, "cutoff_confirm_min_spacing_ms", c.cutoff_confirm_min_spacing_ms);
    write_num(os, "min_plausible_temp_c", c.min_plausible_temp_c);
    write_num(os, "max_plausible_temp_c", c.max_plausible_temp_c);
    write_num(os, "max_temp_rate_c_per_s", c.max_temp_rate_c_per_s);
    write_bool(os, "temp_channel_fallback", c.temp_channel_fallback);
    write_bool(os, "temperature_enabled", c.temperature_enabled);
    os << '\n';

    os << "[pid_tau_only]\n";
    write_bool(os, "pid_enabled", c.pid_enabled);
    write_num(os, "pid_setpoint", c.pid_setpoint);
    write_bool(os, "pid_amplitude", c.pid_amplitude);
    write_bool(os, "pid_duration", c.pid_duration);
    write_bool(os, "pid_duty", c.pid_duty);
    write_bool(os, "pid_interval", c.pid_interval);
    write_num(os, "pid_kp", c.pid_kp);
    write_num(os, "pid_ki", c.pid_ki);
    write_num(os, "pid_kd", c.pid_kd);
    write_num(os, "pid_prediction_tau_s", c.pid_prediction_tau_s);
    write_num(os, "pid_prediction_horizon_s", c.pid_prediction_horizon_s);
    os << '\n';

    os << "[cycling]\n";
    write_bool(os, "use_cycling", c.use_cycling);
    write_num(os, "heating_s", c.heating_s);
    write_num(os, "cooling_s", c.cooling_s);
    write_str(os, "cooling_mode", to_string(c.cooling_mode));
    write_num(os, "cooling_hold_temp", c.cooling_hold_temp);
    os << '\n';

    os << "[connections]\n";
    write_str(os, "com3_port", c.com3_port);
    write_str(os, "com11_port", c.com11_port);
    write_str(os, "temp_channel", temp_channel_to_string(c.temp_channel));
    write_str(os, "udp_host", c.udp_host);
    write_int(os, "udp_port", static_cast<int>(c.udp_port));
    write_bool(os, "simulate_temp", c.simulate_temp);
    write_bool(os, "simulate_us", c.simulate_us);
    os << '\n';

    os << "[paths]\n";
    write_str(os, "auto_save_dir", c.auto_save_dir);
    os << '\n';

    os << "[long_run_protection]\n";
    write_bool(os, "persistent_com3", c.persistent_com3);
    write_bool(os, "communication_retry", c.communication_retry);
    write_int(os, "communication_retry_attempts", c.communication_retry_attempts);
    write_int(os, "communication_retry_initial_backoff_ms", c.communication_retry_initial_backoff_ms);
    write_int(os, "emergency_stop_repeats", c.emergency_stop_repeats);
    write_int(os, "watchdog_timeout_ms", c.watchdog_timeout_ms);
    return os.str();
}

void save_config_file(const std::filesystem::path& path, const Config& config, bool include_comments) {
    std::filesystem::create_directories(path.parent_path().empty() ? std::filesystem::current_path() : path.parent_path());
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) throw std::runtime_error("Cannot write configuration file: " + path.string());
    f << config_to_text(config, include_comments);
}

void write_config_template(const std::filesystem::path& path) {
    Config c;
    c.config_source_type = "template";
    save_config_file(path, c, true);
}

} // namespace sonocontrol
