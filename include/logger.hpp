#pragma once

#include "config.hpp"

#include <array>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sonocontrol {

class ExperimentLogger {
public:
    ExperimentLogger();
    ~ExperimentLogger();

    void start_session(const ActiveParams& initial_params, const Config& config = Config{});
    void end_session();
    void log_temperature(double temp, std::optional<double> t1 = std::nullopt, std::optional<double> t2 = std::nullopt);
    void log_params(const ActiveParams& params);
    void log_event(const std::string& message);
    // Append to the in-memory event history (capped, FIFO) without writing a
    // CSV row. Use for chatty console output (per-packet hex dumps, TIME/TEMP
    // pings) so 24-hour runs don't grow the CSV by hundreds of MB.
    void log_console_only(const std::string& message);
    void log_error(const std::string& message);
    void flush();

    // Setters for per-sample experimental state carried in dedicated CSV
    // columns. Each call updates the logger's "current state" snapshot; every
    // subsequent row written by log_temperature / log_event / log_params /
    // log_error reflects the most recently set value. Callers update these
    // at the point where the value changes (phase transitions, after each
    // PID compute, on transmit/stop) rather than re-passing them on every
    // log call. This keeps per-sample data (setpoint, PID demand, phase, US
    // active) recorded as structured CSV columns instead of free-form event
    // text — which is what `log_console_quiet` deliberately drops.
    void set_phase(const std::string& phase);
    void set_setpoint(std::optional<double> setpoint_c);
    void set_pid_demand(std::optional<double> demand_fraction);
    void set_ultrasound_active(bool active);

    void export_csv_file(const std::filesystem::path& path) const { export_csv(path); }
    const std::filesystem::path& log_dir() const { return log_dir_; }
    const std::filesystem::path& csv_path() const { return csv_path_; }
    const std::filesystem::path& meta_path() const { return meta_path_; }
    size_t error_count() const { return errors_.size(); }
    std::string error_summary_text() const;

private:
    struct Row {
        std::string timestamp;
        double time_s = 0.0;
        std::optional<double> t1;
        std::optional<double> t2;
        std::optional<double> temp;
        ActiveParams params;
        std::string event;
        // Snapshot of the per-sample experiment state at the moment the row
        // was created. All four are independent of params and ActiveParams.
        std::string phase;
        std::optional<double> setpoint_c;
        std::optional<double> pid_demand_fraction;
        bool us_active = false;
    };

    Row make_row(std::optional<double> temp, const std::string& event,
                 std::optional<double> t1 = std::nullopt,
                 std::optional<double> t2 = std::nullopt) const;
    void write_header();
    void write_row(const Row& r);
    void maybe_flush();
    void push_event_bounded(double rel_s, std::string message);
    void save_meta(const std::filesystem::path& path) const;
    void export_csv(const std::filesystem::path& path) const;

    std::filesystem::path log_dir_;
    std::filesystem::path csv_path_;
    std::filesystem::path meta_path_;
    std::ofstream csv_stream_;
    // Bounded event/error history. save_meta() walks these on each rewrite,
    // so an unbounded growth turns long runs into O(N²) total meta work and
    // hundreds of MB of JSON. FIFO eviction keeps the most recent events.
    static constexpr size_t kMaxEventsRetained = 4096;
    static constexpr size_t kMaxErrorsRetained = 256;
    std::deque<std::pair<double, std::string>> events_;
    std::deque<std::pair<double, std::string>> errors_;
    size_t events_dropped_ = 0;
    size_t errors_dropped_ = 0;
    std::chrono::system_clock::time_point start_wall_{};
    std::chrono::steady_clock::time_point start_steady_{};
    std::chrono::steady_clock::time_point last_flush_{};
    std::chrono::steady_clock::time_point last_meta_save_{};
    bool started_ = false;
    std::string session_id_;
    ActiveParams initial_params_{};
    ActiveParams active_params_{};
    // Current per-sample experiment state. Updated via the set_* methods and
    // copied into every Row that the logger writes.
    std::string active_phase_;
    std::optional<double> active_setpoint_c_;
    std::optional<double> active_pid_demand_;
    bool active_us_active_ = false;
    Config session_config_{};
    size_t total_rows_ = 0;
    size_t total_samples_ = 0;
    int flush_interval_s_ = 5;
    // Rewriting the JSON metadata is O(events+errors). Spacing it out from
    // the CSV stream flush keeps the per-flush cost bounded for long runs.
    int meta_save_interval_s_ = 60;
    // Large I/O buffer cuts write() syscalls ~8× compared to the default 4 KB.
    std::array<char, 32 * 1024> csv_buffer_{};
};

std::string now_hms();
std::string timestamp_ms(std::chrono::system_clock::time_point tp);

} // namespace sonocontrol
