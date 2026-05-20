#include "temperature.hpp"
#include "logger.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace sonocontrol {

HH806AUSensor::HH806AUSensor(std::string port) : HH806AUSensor(std::move(port), 10.0, 80.0, 15.0) {}

HH806AUSensor::HH806AUSensor(std::string port, double min_temp_c, double max_temp_c, double max_rate_c_per_s)
    : port_(std::move(port)),
      min_temp_c_(std::min(min_temp_c, max_temp_c)),
      max_temp_c_(std::max(min_temp_c, max_temp_c)),
      max_rate_c_per_s_(std::max(1.0, max_rate_c_per_s)) {}

bool HH806AUSensor::ensure_open() {
    if (serial_.is_open()) return true;
    const bool ok = serial_.open(port_, 19200, 8, 'E', 1, 700);
    if (ok) {
        // Clear stale bytes emitted by the meter or USB-serial driver after opening.
        serial_.reset_input_buffer();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        (void)serial_.read_available(256);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        (void)serial_.read_available(256);
        serial_.reset_input_buffer();
    }
    return ok;
}

std::optional<double> HH806AUSensor::decode_pair(const std::vector<uint8_t>& bytes,
                                                        size_t offset,
                                                        ByteOrder order,
                                                        const std::string& channel,
                                                        const std::vector<uint8_t>& frame) const {
    if (offset + 1 >= bytes.size()) return std::nullopt;
    uint16_t raw = 0;
    if (order == ByteOrder::BigEndian) {
        raw = static_cast<uint16_t>((static_cast<uint16_t>(bytes[offset]) << 8u) | bytes[offset + 1]);
    } else {
        raw = static_cast<uint16_t>((static_cast<uint16_t>(bytes[offset + 1]) << 8u) | bytes[offset]);
    }
    if (raw == 0x7FFFu || raw == 0x0000u) return std::nullopt;
    const double val = static_cast<double>(raw) / 10.0;
    if (val >= min_temp_c_ && val <= max_temp_c_) return val;
    log_raw_issue(channel + " decode out of plausible range: " + std::to_string(val) +
                  " C offset=" + std::to_string(offset) +
                  (order == ByteOrder::BigEndian ? " BE" : " LE"), frame);
    return std::nullopt;
}

std::string HH806AUSensor::hex_dump(const std::vector<uint8_t>& bytes) {
    static constexpr char lut[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(bytes.size() * 3);
    for (uint8_t b : bytes) {
        if (!out.empty()) out.push_back(' ');
        out.push_back(lut[(b >> 4) & 0x0F]);
        out.push_back(lut[b & 0x0F]);
    }
    return out;
}

void HH806AUSensor::log_raw_issue(const std::string& reason, const std::vector<uint8_t>& bytes) const {
    try {
        static std::mutex log_mutex;
        std::lock_guard<std::mutex> g(log_mutex);
        const auto dir = std::filesystem::current_path() / "logs";
        std::filesystem::create_directories(dir);
        std::ofstream f(dir / "hh806au_raw.log", std::ios::out | std::ios::app);
        if (!f) return;
        f << timestamp_ms(std::chrono::system_clock::now())
          << " port=" << port_
          << " reason=" << reason
          << " bytes=" << hex_dump(bytes) << '\n';
    } catch (...) {
        // Diagnostic logging must never break temperature acquisition.
    }
}

std::pair<std::optional<double>, std::optional<double>> HH806AUSensor::parse_response(const std::vector<uint8_t>& bytes) const {
    if (bytes.size() < 14) {
        log_raw_issue("short response (<14 bytes)", bytes);
        return {std::nullopt, std::nullopt};
    }

    // V7 policy: accept only the confirmed HH806AU measurement frame.
    // Valid inserted-probe examples observed in raw logs look like:
    //   3E 0F 00 00 10 00 CB 14 01 10 00 BC 0B 01
    // where T1 = bytes[5:7] BE / 10 and T2 = bytes[10:12] BE / 10.
    // Unplugged/status/noise sequences may contain plausible-looking byte pairs such as
    // 0x0200 -> 51.2 C. Therefore a pair is not considered a temperature unless it is
    // inside a frame that starts with 3E 0F and the per-channel status byte indicates a
    // connected thermocouple.
    constexpr uint8_t kHeader0 = 0x3E;
    constexpr uint8_t kHeader1 = 0x0F;
    constexpr uint8_t kConnectedStatus = 0x10;

    auto parse_at = [&](size_t offset) -> std::pair<std::optional<double>, std::optional<double>> {
        if (offset + 14 > bytes.size()) return {std::nullopt, std::nullopt};
        std::vector<uint8_t> frame(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                                   bytes.begin() + static_cast<std::ptrdiff_t>(offset + 14));
        if (frame[0] != kHeader0 || frame[1] != kHeader1) {
            return {std::nullopt, std::nullopt};
        }

        std::optional<double> t1;
        std::optional<double> t2;
        if (frame[4] == kConnectedStatus) {
            t1 = decode_pair(frame, 5, ByteOrder::BigEndian, "T1 primary14", bytes);
        } else {
            log_raw_issue("T1 not connected/status byte not valid: 0x" + hex_dump({frame[4]}), bytes);
        }
        if (frame[9] == kConnectedStatus) {
            t2 = decode_pair(frame, 10, ByteOrder::BigEndian, "T2 primary14", bytes);
        } else {
            log_raw_issue("T2 not connected/status byte not valid: 0x" + hex_dump({frame[9]}), bytes);
        }
        if (t1 || t2) {
            if (offset != 0) {
                log_raw_issue("accepted HH806AU frame after header resync at offset " + std::to_string(offset), bytes);
            } else if (bytes.size() > 14) {
                log_raw_issue("accepted primary 14-byte HH806AU frame; ignored trailing diagnostic bytes", bytes);
            }
        }
        return {t1, t2};
    };

    // Fast path: the normal response starts with 3E 0F.
    auto parsed = parse_at(0);
    if (parsed.first || parsed.second) return parsed;

    // Recovery path: if the driver delivered stale leading bytes, resync only on the
    // confirmed 3E 0F header. Never accept arbitrary shifted plausible pairs.
    for (size_t i = 1; i + 14 <= bytes.size(); ++i) {
        if (bytes[i] == kHeader0 && bytes[i + 1] == kHeader1) {
            parsed = parse_at(i);
            if (parsed.first || parsed.second) return parsed;
        }
    }

    // Diagnostic-only scan. These candidates are intentionally not returned to PID/cutoff.
    std::ostringstream oss;
    oss << "no valid 3E 0F HH806AU measurement frame; diagnostic plausible pairs only:";
    for (size_t i = 0; i + 1 < bytes.size(); ++i) {
        const uint16_t be = static_cast<uint16_t>((static_cast<uint16_t>(bytes[i]) << 8u) | bytes[i + 1]);
        const uint16_t le = static_cast<uint16_t>((static_cast<uint16_t>(bytes[i + 1]) << 8u) | bytes[i]);
        const double vbe = be / 10.0;
        const double vle = le / 10.0;
        if (vbe >= min_temp_c_ && vbe <= max_temp_c_) oss << " off" << i << "BE=" << vbe;
        if (vle >= min_temp_c_ && vle <= max_temp_c_) oss << " off" << i << "LE=" << vle;
    }
    log_raw_issue(oss.str(), bytes);
    return {std::nullopt, std::nullopt};
}

std::pair<std::optional<double>, std::optional<double>> HH806AUSensor::apply_plausibility_filter(std::optional<double> t1, std::optional<double> t2, const std::vector<uint8_t>& frame) {
    const auto now = std::chrono::steady_clock::now();
    const double dt = last_valid_time_.time_since_epoch().count() == 0
        ? 1.0
        : std::max(0.05, std::chrono::duration<double>(now - last_valid_time_).count());

    auto filter_one = [&](std::optional<double> value, std::optional<double> last, const std::string& channel) -> std::optional<double> {
        if (!value) return std::nullopt;
        if (*value < min_temp_c_ || *value > max_temp_c_) {
            log_raw_issue(channel + " filtered out of configured range: " + std::to_string(*value) + " C", frame);
            return std::nullopt;
        }
        if (last) {
            const double jump = std::abs(*value - *last);
            const double allowed = std::max(1.5, max_rate_c_per_s_ * dt);
            if (jump > allowed) {
                log_raw_issue(channel + " rate-limit reject: prev=" + std::to_string(*last) +
                              " now=" + std::to_string(*value) + " dt=" + std::to_string(dt) +
                              " allowed_jump=" + std::to_string(allowed), frame);
                return std::nullopt;
            }
        }
        return value;
    };

    t1 = filter_one(t1, last_t1_, "T1");
    t2 = filter_one(t2, last_t2_, "T2");
    if (t1 || t2) {
        if (t1) last_t1_ = t1;
        if (t2) last_t2_ = t2;
        last_valid_time_ = now;
    }
    return {t1, t2};
}

std::pair<std::optional<double>, std::optional<double>> HH806AUSensor::read() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensure_open()) return {std::nullopt, std::nullopt};

    // Avoid over-polling. HH806AU reads faster than about 3 Hz are unnecessary for the
    // current PID loop and increase partial USB-serial chunk risk on some PCs.
    const auto now0 = std::chrono::steady_clock::now();
    if (last_query_.time_since_epoch().count() != 0) {
        const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now0 - last_query_).count();
        if (since < 300) std::this_thread::sleep_for(std::chrono::milliseconds(300 - since));
    }
    last_query_ = std::chrono::steady_clock::now();

    serial_.reset_input_buffer();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const auto cmd = reinterpret_cast<const uint8_t*>("#0A0000NA2\r\n");
    if (!serial_.write_all(cmd, 12)) {
        serial_.close();
        return {std::nullopt, std::nullopt};
    }

    // Read the same 14 bytes expected by the original Python implementation, then collect
    // any immediately available tail bytes only for alignment diagnostics.
    std::vector<uint8_t> buf = serial_.read_exact(14);
    const auto tail_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(40);
    while (std::chrono::steady_clock::now() < tail_deadline && buf.size() < 32) {
        auto chunk = serial_.read_available(32 - buf.size());
        if (!chunk.empty()) {
            buf.insert(buf.end(), chunk.begin(), chunk.end());
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    if (buf.empty()) {
        log_raw_issue("empty HH806AU response", buf);
        return {std::nullopt, std::nullopt};
    }

    const auto parsed = parse_response(buf);
    const auto filtered = apply_plausibility_filter(parsed.first, parsed.second, buf);
    if (filtered.first || filtered.second) return filtered;

    log_raw_issue("read failed after Python-compatible parse and plausibility filter", buf);
    return {std::nullopt, std::nullopt};
}

void HH806AUSensor::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    serial_.close();
}

TemperatureSimulator::TemperatureSimulator(double baseline)
    : temp_(baseline), baseline_(baseline), last_t_(Clock::now()), pulse_end_(last_t_),
      rng_(std::random_device{}()) {}

void TemperatureSimulator::set_ultrasound_params(double amplitude, double duty_cycle, int duration_ms, double interval_s) {
    const auto now = Clock::now();
    integrate_until(now);

    amplitude_ = std::clamp(amplitude, 0.0, 1.0);
    duty_cycle_ = std::clamp(duty_cycle, 0.0, 1.0);
    duration_ms_ = std::max(0, duration_ms);
    interval_s_ = std::max(0.001, interval_s);

    us_active_ = amplitude_ > 0.0 && duty_cycle_ > 0.0 && duration_ms_ > 0;
    pulse_end_ = now + std::chrono::milliseconds(duration_ms_);
}

void TemperatureSimulator::set_ultrasound_active(bool active) {
    const auto now = Clock::now();
    integrate_until(now);

    us_active_ = active;
    if (!active) {
        amplitude_ = 0.0;
        duty_cycle_ = 0.0;
        pulse_end_ = now;
    }
}

void TemperatureSimulator::integrate_until(Clock::time_point now) {
    static constexpr double MAX_HEAT_C_PER_S = 12.0;
    static constexpr double COOL_RATE = 0.08;

    if (now <= last_t_) return;
    double remaining = std::min(2.0, std::chrono::duration<double>(now - last_t_).count());
    auto cursor = now - std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(remaining));

    while (remaining > 1e-6) {
        const bool heating = us_active_ && cursor < pulse_end_;
        double step = std::min(0.05, remaining);
        if (heating) {
            const double until_end = std::chrono::duration<double>(pulse_end_ - cursor).count();
            step = std::min(step, std::max(0.0, until_end));
            if (step <= 1e-6) {
                us_active_ = false;
                continue;
            }
        }

        const double power = heating ? (amplitude_ * amplitude_ * duty_cycle_) : 0.0;
        const double dtemp = (MAX_HEAT_C_PER_S * power - COOL_RATE * (temp_ - baseline_)) * step;
        temp_ += dtemp;
        temp_ = std::clamp(temp_, baseline_ - 2.0, 48.0);

        cursor += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(step));
        remaining -= step;
        if (cursor >= pulse_end_) us_active_ = false;
    }
    last_t_ = now;
}

std::pair<std::optional<double>, std::optional<double>> TemperatureSimulator::read() {
    const auto now = Clock::now();
    integrate_until(now);

    const double n1 = noise_(rng_);
    const double n2 = noise_(rng_);
    const double t1 = std::round((temp_ - 0.03 + n1) * 100.0) / 100.0;
    const double t2 = std::round((temp_ + 0.03 + n2) * 100.0) / 100.0;
    return {t1, t2};
}

} // namespace sonocontrol
