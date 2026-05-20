#pragma once

#include "config.hpp"

#include <chrono>
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
    void log_error(const std::string& message);
    void flush();

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
    };

    Row make_row(std::optional<double> temp, const std::string& event,
                 std::optional<double> t1 = std::nullopt,
                 std::optional<double> t2 = std::nullopt) const;
    void write_header();
    void write_row(const Row& r);
    void maybe_flush();
    void save_meta(const std::filesystem::path& path) const;
    void export_csv(const std::filesystem::path& path) const;

    std::filesystem::path log_dir_;
    std::filesystem::path csv_path_;
    std::filesystem::path meta_path_;
    std::ofstream csv_stream_;
    std::vector<std::pair<double, std::string>> events_;
    std::vector<std::pair<double, std::string>> errors_;
    std::chrono::system_clock::time_point start_wall_{};
    std::chrono::steady_clock::time_point start_steady_{};
    std::chrono::steady_clock::time_point last_flush_{};
    bool started_ = false;
    std::string session_id_;
    ActiveParams initial_params_{};
    ActiveParams active_params_{};
    Config session_config_{};
    size_t total_rows_ = 0;
    size_t total_samples_ = 0;
    int flush_interval_s_ = 5;
};

std::string now_hms();
std::string timestamp_ms(std::chrono::system_clock::time_point tp);

} // namespace sonocontrol
