#pragma once

#include "serial_port.hpp"

#include <chrono>
#include <optional>
#include <mutex>
#include <filesystem>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace sonocontrol {

class ITemperatureSensor {
public:
    virtual ~ITemperatureSensor() = default;
    virtual std::pair<std::optional<double>, std::optional<double>> read() = 0;
    virtual void close() {}
    virtual bool test_connection() { auto [t1, t2] = read(); return t1.has_value() || t2.has_value(); }
    virtual void set_ultrasound_params(double, double, int, double) {}
    virtual void set_ultrasound_active(bool) {}
};

class NullTemperatureSensor final : public ITemperatureSensor {
public:
    std::pair<std::optional<double>, std::optional<double>> read() override { return {std::nullopt, std::nullopt}; }
    bool test_connection() override { return true; }
};

class HH806AUSensor final : public ITemperatureSensor {
public:
    explicit HH806AUSensor(std::string port = "COM5");
    HH806AUSensor(std::string port, double min_temp_c, double max_temp_c, double max_rate_c_per_s);
    std::pair<std::optional<double>, std::optional<double>> read() override;
    void close() override;

private:
    bool ensure_open();
    enum class ByteOrder { BigEndian, LittleEndian };
    struct LayoutProfile {
        const char* name;
        size_t t1_offset;
        size_t t2_offset;
        ByteOrder order;
        size_t min_size;
    };
    std::optional<double> decode_pair(const std::vector<uint8_t>& bytes, size_t offset, ByteOrder order, const std::string& channel, const std::vector<uint8_t>& frame) const;
    static std::string hex_dump(const std::vector<uint8_t>& bytes);
    std::pair<std::optional<double>, std::optional<double>> parse_response(const std::vector<uint8_t>& bytes) const;
    std::pair<std::optional<double>, std::optional<double>> apply_plausibility_filter(std::optional<double> t1, std::optional<double> t2, const std::vector<uint8_t>& frame);
    void log_raw_issue(const std::string& reason, const std::vector<uint8_t>& bytes) const;

    std::string port_;
    double min_temp_c_ = 10.0;
    double max_temp_c_ = 80.0;
    double max_rate_c_per_s_ = 15.0;
    SerialPort serial_;
    std::mutex mutex_;
    std::chrono::steady_clock::time_point last_query_{};
    std::chrono::steady_clock::time_point last_valid_time_{};
    std::optional<double> last_t1_;
    std::optional<double> last_t2_;
    mutable std::string selected_layout_;
};

class TemperatureSimulator final : public ITemperatureSensor {
public:
    explicit TemperatureSimulator(double baseline = 37.0);
    std::pair<std::optional<double>, std::optional<double>> read() override;
    void set_ultrasound_params(double amplitude, double duty_cycle, int duration_ms, double interval_s) override;
    void set_ultrasound_active(bool active) override;
    bool test_connection() override { return true; }

private:
    using Clock = std::chrono::steady_clock;
    void integrate_until(Clock::time_point now);

    double temp_ = 37.0;
    double baseline_ = 37.0;
    Clock::time_point last_t_{};
    double amplitude_ = 0.0;
    double duty_cycle_ = 0.0;
    int duration_ms_ = 0;
    double interval_s_ = 1.0;
    bool us_active_ = false;
    Clock::time_point pulse_end_{};

    std::mt19937 rng_;
    std::normal_distribution<double> noise_{0.0, 0.02};
};

} // namespace sonocontrol
