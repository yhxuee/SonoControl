#include "logger.hpp"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace sonocontrol {

namespace {
std::string csv_escape(const std::string& s) {
    if (s.find_first_of(",\"\n\r") == std::string::npos) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out += c;
    }
    out += "\"";
    return out;
}

std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    return out;
}

std::string fmt_double(double v, int precision) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << v;
    return os.str();
}

std::filesystem::path default_log_dir() {
    auto d = std::filesystem::current_path() / "logs";
    std::filesystem::create_directories(d);
    return d;
}

std::string session_id_from_time(std::chrono::system_clock::time_point tp) {
    const auto t = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return os.str();
}

} // namespace

std::string now_hms() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream os;
    os << std::put_time(&tm, "%H:%M:%S");
    return os.str();
}

std::string timestamp_ms(std::chrono::system_clock::time_point tp) {
    const auto t = std::chrono::system_clock::to_time_t(tp);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count() % 1000;
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream os;
    os << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms;
    return os.str();
}

ExperimentLogger::ExperimentLogger() : log_dir_(default_log_dir()) {}

ExperimentLogger::~ExperimentLogger() {
    try { end_session(); } catch (...) {}
}

void ExperimentLogger::start_session(const ActiveParams& initial_params, const Config& config) {
    if (started_) end_session();
    events_.clear();
    errors_.clear();
    total_rows_ = 0;
    total_samples_ = 0;
    start_wall_ = std::chrono::system_clock::now();
    start_steady_ = std::chrono::steady_clock::now();
    last_flush_ = start_steady_;
    started_ = true;
    session_id_ = session_id_from_time(start_wall_);
    initial_params_ = initial_params;
    active_params_ = initial_params;
    session_config_ = config;
    csv_path_ = log_dir_ / (session_id_ + "_log.csv");
    meta_path_ = log_dir_ / (session_id_ + "_meta.json");
    csv_stream_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_stream_) {
        started_ = false;
        throw std::runtime_error("Cannot open streaming log file: " + csv_path_.string());
    }
    write_header();
    log_event("Session started");
    flush();
}

void ExperimentLogger::end_session() {
    if (!started_) return;
    log_event("Session ended");
    flush();
    save_meta(meta_path_);
    csv_stream_.close();
    started_ = false;
}

void ExperimentLogger::log_temperature(double temp, std::optional<double> t1, std::optional<double> t2) {
    if (!started_) return;
    ++total_samples_;
    write_row(make_row(temp, "", t1, t2));
    maybe_flush();
}

void ExperimentLogger::log_params(const ActiveParams& params) {
    if (!started_) return;
    active_params_ = params;
    write_row(make_row(std::nullopt, "US params updated"));
    maybe_flush();
}

void ExperimentLogger::log_event(const std::string& message) {
    if (!started_) return;
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    events_.push_back({rel, message});
    write_row(make_row(std::nullopt, message));
    maybe_flush();
}

void ExperimentLogger::log_error(const std::string& message) {
    if (!started_) return;
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    errors_.push_back({rel, message});
    events_.push_back({rel, std::string("ERROR: ") + message});
    write_row(make_row(std::nullopt, std::string("ERROR: ") + message));
    flush();
}

void ExperimentLogger::write_header() {
    csv_stream_ << "timestamp,time_s,T1_C,T2_C,temp_C,amplitude,cfreq_kHz,prf_Hz,duty_pct,duration_ms,interval_s,wave_shape,config_type,config_file,event\n";
}

void ExperimentLogger::write_row(const Row& r) {
    if (!csv_stream_) return;
    csv_stream_ << r.timestamp << ','
                << fmt_double(r.time_s, 2) << ','
                << (r.t1 ? fmt_double(*r.t1, 3) : "") << ','
                << (r.t2 ? fmt_double(*r.t2, 3) : "") << ','
                << (r.temp ? fmt_double(*r.temp, 3) : "") << ','
                << fmt_double(r.params.amplitude, 3) << ','
                << fmt_double(r.params.cfreq_hz / 1000.0, 1) << ','
                << fmt_double(r.params.prf_hz, 0) << ','
                << fmt_double(r.params.duty_cycle * 100.0, 1) << ','
                << r.params.duration_ms << ','
                << fmt_double(r.params.interval_time_s, 2) << ','
                << to_string(r.params.wave_shape) << ','
                << csv_escape(session_config_.config_source_type) << ','
                << csv_escape(session_config_.config_file_path) << ','
                << csv_escape(r.event) << '\n';
    ++total_rows_;
}

void ExperimentLogger::maybe_flush() {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_flush_).count() >= flush_interval_s_) {
        flush();
    }
}

void ExperimentLogger::flush() {
    if (csv_stream_) csv_stream_.flush();
    last_flush_ = std::chrono::steady_clock::now();
    if (started_ && !meta_path_.empty()) save_meta(meta_path_);
}

ExperimentLogger::Row ExperimentLogger::make_row(std::optional<double> temp,
                                                 const std::string& event,
                                                 std::optional<double> t1,
                                                 std::optional<double> t2) const {
    const auto now = std::chrono::system_clock::now();
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    return Row{timestamp_ms(now), rel, t1, t2, temp, active_params_, event};
}

void ExperimentLogger::export_csv(const std::filesystem::path& path) const {
    if (!csv_path_.empty() && std::filesystem::exists(csv_path_)) {
        std::filesystem::copy_file(csv_path_, path, std::filesystem::copy_options::overwrite_existing);
        return;
    }
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    f << "timestamp,time_s,T1_C,T2_C,temp_C,amplitude,cfreq_kHz,prf_Hz,duty_pct,duration_ms,interval_s,wave_shape,config_type,config_file,event\n";
}

void ExperimentLogger::save_meta(const std::filesystem::path& path) const {
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    const double duration_s = started_
        ? std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count()
        : 0.0;

    f << "{\n";
    f << "  \"session_id\": \"" << json_escape(session_id_) << "\",\n";
    f << "  \"start_local\": \"" << timestamp_ms(start_wall_) << "\",\n";
    f << "  \"duration_s\": " << fmt_double(duration_s, 2) << ",\n";
    f << "  \"csv_path\": \"" << json_escape(csv_path_.string()) << "\",\n";
    f << "  \"streaming_flush_interval_s\": " << flush_interval_s_ << ",\n";
    f << "  \"config_source_type\": \"" << json_escape(session_config_.config_source_type) << "\",\n";
    f << "  \"config_file_path\": \"" << json_escape(session_config_.config_file_path) << "\",\n";
    f << "  \"auto_save_dir\": \"" << json_escape(session_config_.auto_save_dir) << "\",\n";
    f << "  \"pid_prediction_tau_s\": " << fmt_double(session_config_.pid_prediction_tau_s, 3) << ",\n";
    f << "  \"pid_prediction_horizon_s\": " << fmt_double(session_config_.pid_prediction_horizon_s, 3) << ",\n";
    f << "  \"pid_prediction_model\": \"T_future=T+tau*dTdt*(1-exp(-t1/tau))\",\n";
    f << "  \"initial_params\": {\n";
    f << "    \"amplitude\": " << fmt_double(initial_params_.amplitude, 3) << ",\n";
    f << "    \"cfreq_hz\": " << fmt_double(initial_params_.cfreq_hz, 1) << ",\n";
    f << "    \"prf_hz\": " << fmt_double(initial_params_.prf_hz, 0) << ",\n";
    f << "    \"duty_cycle\": " << fmt_double(initial_params_.duty_cycle, 3) << ",\n";
    f << "    \"duration_ms\": " << initial_params_.duration_ms << ",\n";
    f << "    \"interval_time_s\": " << fmt_double(initial_params_.interval_time_s, 2) << ",\n";
    f << "    \"wave_shape\": \"" << to_string(initial_params_.wave_shape) << "\"\n";
    f << "  },\n";
    f << "  \"events\": [\n";
    for (size_t i = 0; i < events_.size(); ++i) {
        f << "    [" << fmt_double(events_[i].first, 2) << ", \"" << json_escape(events_[i].second) << "\"]";
        if (i + 1 < events_.size()) f << ',';
        f << '\n';
    }
    f << "  ],\n";
    f << "  \"errors\": [\n";
    for (size_t i = 0; i < errors_.size(); ++i) {
        f << "    [" << fmt_double(errors_[i].first, 2) << ", \"" << json_escape(errors_[i].second) << "\"]";
        if (i + 1 < errors_.size()) f << ',';
        f << '\n';
    }
    f << "  ],\n";
    f << "  \"total_rows\": " << total_rows_ << ",\n";
    f << "  \"total_samples\": " << total_samples_ << ",\n";
    f << "  \"error_count\": " << errors_.size() << "\n";
    f << "}\n";
}

std::string ExperimentLogger::error_summary_text() const {
    std::ostringstream os;
    os << "Errors: " << errors_.size();
    if (!errors_.empty()) {
        os << "\n";
        for (const auto& [t, msg] : errors_) {
            os << "- t=" << fmt_double(t, 2) << "s: " << msg << "\n";
        }
    }
    return os.str();
}

} // namespace sonocontrol
