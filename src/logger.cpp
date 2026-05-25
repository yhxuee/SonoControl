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
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count() % 1000;
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream os;
    os << std::put_time(&tm, "%Y%m%d_%H%M%S")
       << '_' << std::setw(3) << std::setfill('0') << ms;
    return os.str();
}

// Returns a session ID guaranteed not to clash with an existing
// <log_dir>/<id>_log.csv. We start from the timestamp-with-milliseconds form
// above (so two sessions launched a few ms apart already get distinct IDs);
// if a file with that name somehow already exists — clock skew, a sequence
// run that hits the same wall time twice, an operator restoring an old log
// folder — we suffix `_2`, `_3`, … until the slot is free. The probe is
// O(1) in normal use because the timestamp resolution is 1 ms.
std::string unique_session_id(const std::filesystem::path& log_dir,
                              std::chrono::system_clock::time_point tp) {
    const std::string base = session_id_from_time(tp);
    if (!std::filesystem::exists(log_dir / (base + "_log.csv"))) return base;
    for (int n = 2; n < 1000; ++n) {
        std::string candidate = base + "_" + std::to_string(n);
        if (!std::filesystem::exists(log_dir / (candidate + "_log.csv"))) return candidate;
    }
    // Pathological fallback (≥1000 collisions in the same millisecond): tack
    // on the lower bits of the steady clock. Still throws downstream if the
    // file is also taken, which is the desired loud failure.
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return base + "_x" + std::to_string(now & 0xFFFF);
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
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (started_) end_session();
    events_.clear();
    errors_.clear();
    events_dropped_ = 0;
    errors_dropped_ = 0;
    total_rows_ = 0;
    total_samples_ = 0;
    // Reset per-sample experiment-state snapshot for the new session.
    active_phase_.clear();
    active_setpoint_c_.reset();
    active_pid_demand_.reset();
    active_us_active_ = false;
    start_wall_ = std::chrono::system_clock::now();
    start_steady_ = std::chrono::steady_clock::now();
    last_flush_ = start_steady_;
    last_meta_save_ = start_steady_;
    started_ = true;
    session_id_ = unique_session_id(log_dir_, start_wall_);
    initial_params_ = initial_params;
    active_params_ = initial_params;
    session_config_ = config;
    csv_path_ = log_dir_ / (session_id_ + "_log.csv");
    meta_path_ = log_dir_ / (session_id_ + "_meta.json");
    // pubsetbuf must be called before open() for portable effect; the larger
    // streambuf reduces write() syscalls ~8× compared to the default 4 KB.
    // The 5 s explicit flush() still bounds crash data loss to one window.
    csv_stream_.rdbuf()->pubsetbuf(csv_buffer_.data(), static_cast<std::streamsize>(csv_buffer_.size()));
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
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (!started_) return;
    log_event("Session ended");
    if (csv_stream_) csv_stream_.flush();
    last_flush_ = std::chrono::steady_clock::now();
    // Force a final meta rewrite even if the slow-cadence timer hasn't fired,
    // so the on-disk JSON matches the final state of the session.
    save_meta(meta_path_);
    last_meta_save_ = last_flush_;
    csv_stream_.close();
    started_ = false;
}

void ExperimentLogger::log_temperature(double temp, std::optional<double> t1, std::optional<double> t2) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (!started_) return;
    ++total_samples_;
    write_row(make_row(temp, "", t1, t2));
    maybe_flush();
}

void ExperimentLogger::log_params(const ActiveParams& params) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (!started_) return;
    active_params_ = params;
    write_row(make_row(std::nullopt, "US params updated"));
    maybe_flush();
}

void ExperimentLogger::log_event(const std::string& message) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (!started_) return;
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    push_event_bounded(rel, message);
    write_row(make_row(std::nullopt, message));
    maybe_flush();
}

void ExperimentLogger::log_console_only(const std::string& message) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (!started_) return;
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    push_event_bounded(rel, message);
}

void ExperimentLogger::log_error(const std::string& message) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (!started_) return;
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    if (errors_.size() >= kMaxErrorsRetained) {
        errors_.pop_front();
        ++errors_dropped_;
    }
    errors_.push_back({rel, message});
    push_event_bounded(rel, std::string("ERROR: ") + message);
    write_row(make_row(std::nullopt, std::string("ERROR: ") + message));
    // Errors are rare and operationally important — persist meta immediately
    // so a subsequent crash still has a record of what went wrong.
    if (csv_stream_) csv_stream_.flush();
    last_flush_ = std::chrono::steady_clock::now();
    if (!meta_path_.empty()) save_meta(meta_path_);
    last_meta_save_ = last_flush_;
}

void ExperimentLogger::write_header() {
    // Schema-stable columns come first; new per-sample experiment-state
    // columns are appended at the end so column-index-based readers of the
    // previous CSV continue to work for the columns they already knew about.
    csv_stream_ << "timestamp,time_s,T1_C,T2_C,temp_C,amplitude,cfreq_kHz,prf_Hz,duty_pct,duration_ms,interval_s,wave_shape,config_type,config_file,event,phase,setpoint_C,pid_demand_pct,us_active\n";
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
                << csv_escape(r.event) << ','
                << csv_escape(r.phase) << ','
                << (r.setpoint_c ? fmt_double(*r.setpoint_c, 3) : "") << ','
                << (r.pid_demand_fraction ? fmt_double(*r.pid_demand_fraction * 100.0, 2) : "") << ','
                << (r.us_active ? 1 : 0)
                << '\n';
    ++total_rows_;
}

void ExperimentLogger::maybe_flush() {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_flush_).count() >= flush_interval_s_) {
        flush();
    }
}

void ExperimentLogger::push_event_bounded(double rel_s, std::string message) {
    if (events_.size() >= kMaxEventsRetained) {
        events_.pop_front();
        ++events_dropped_;
    }
    events_.push_back({rel_s, std::move(message)});
}

void ExperimentLogger::flush() {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (csv_stream_) csv_stream_.flush();
    const auto now = std::chrono::steady_clock::now();
    last_flush_ = now;
    if (!started_ || meta_path_.empty()) return;
    // save_meta is O(events + errors); on the 5 s flush cadence with a long
    // running session that was the dominant background cost. Rewrite the
    // metadata at a slower cadence than the CSV flush.
    const double since_meta = std::chrono::duration<double>(now - last_meta_save_).count();
    if (since_meta >= static_cast<double>(meta_save_interval_s_)) {
        save_meta(meta_path_);
        last_meta_save_ = now;
    }
}

ExperimentLogger::Row ExperimentLogger::make_row(std::optional<double> temp,
                                                 const std::string& event,
                                                 std::optional<double> t1,
                                                 std::optional<double> t2) const {
    const auto now = std::chrono::system_clock::now();
    const double rel = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_steady_).count();
    Row r{timestamp_ms(now), rel, t1, t2, temp, active_params_, event,
          active_phase_, active_setpoint_c_, active_pid_demand_, active_us_active_};
    return r;
}

void ExperimentLogger::set_phase(const std::string& phase) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    active_phase_ = phase;
}

void ExperimentLogger::set_setpoint(std::optional<double> setpoint_c) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    active_setpoint_c_ = setpoint_c;
}

void ExperimentLogger::set_pid_demand(std::optional<double> demand_fraction) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    active_pid_demand_ = demand_fraction;
}

void ExperimentLogger::set_ultrasound_active(bool active) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    active_us_active_ = active;
}

void ExperimentLogger::export_csv_file(const std::filesystem::path& path) const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    // Flush the writer to disk first so the snapshot taken by copy_file
    // includes whatever the worker just wrote — otherwise the export can
    // miss the last few seconds of rows when called mid-run.
    if (csv_stream_) const_cast<std::ofstream&>(csv_stream_).flush();
    export_csv(path);
}

void ExperimentLogger::export_csv(const std::filesystem::path& path) const {
    if (!csv_path_.empty() && std::filesystem::exists(csv_path_)) {
        std::filesystem::copy_file(csv_path_, path, std::filesystem::copy_options::overwrite_existing);
        return;
    }
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    f << "timestamp,time_s,T1_C,T2_C,temp_C,amplitude,cfreq_kHz,prf_Hz,duty_pct,duration_ms,interval_s,wave_shape,config_type,config_file,event,phase,setpoint_C,pid_demand_pct,us_active\n";
}

size_t ExperimentLogger::error_count() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return errors_.size();
}

std::string ExperimentLogger::session_id() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return started_ ? session_id_ : std::string{};
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
    f << "  \"events_dropped\": " << events_dropped_ << ",\n";
    f << "  \"errors_dropped\": " << errors_dropped_ << ",\n";
    f << "  \"error_count\": " << errors_.size() << "\n";
    f << "}\n";
}

std::string ExperimentLogger::error_summary_text() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
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
