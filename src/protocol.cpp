#include "protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace sonocontrol {

namespace {
std::string lower_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}
}

const char* to_string(WaveShape shape) {
    switch (shape) {
        case WaveShape::Sine: return "sine";
        case WaveShape::Square: return "square";
        case WaveShape::Triangle: return "triangle";
    }
    return "sine";
}

WaveShape wave_shape_from_string(const std::string& input) {
    const auto s = lower_copy(input);
    if (s == "sine" || s == "sin") return WaveShape::Sine;
    if (s == "square" || s == "sq") return WaveShape::Square;
    if (s == "triangle" || s == "tri") return WaveShape::Triangle;
    throw std::invalid_argument("Unsupported waveform shape: " + input);
}

const char* to_string(CoolingMode mode) {
    switch (mode) {
        case CoolingMode::Stop: return "stop";
        case CoolingMode::Low: return "low";
    }
    return "stop";
}

CoolingMode cooling_mode_from_string(const std::string& input) {
    const auto s = lower_copy(input);
    if (s == "stop") return CoolingMode::Stop;
    if (s == "low" || s == "hold") return CoolingMode::Low;
    throw std::invalid_argument("Unsupported cooling mode: " + input);
}

const char* to_string(LengthMode mode) {
    switch (mode) {
        case LengthMode::TotalDuration: return "total_duration";
        case LengthMode::RepeatingCycles: return "repeating_cycles";
        case LengthMode::HoldAfterTarget: return "hold_after_target";
    }
    return "total_duration";
}

LengthMode length_mode_from_string(const std::string& input) {
    const auto s = lower_copy(input);
    if (s == "total" || s == "total_duration" || s == "duration") return LengthMode::TotalDuration;
    if (s == "repeat" || s == "repeating" || s == "repeating_cycles" || s == "cycles") return LengthMode::RepeatingCycles;
    if (s == "hold_after_target" || s == "target_hold" || s == "after_target" || s == "target") return LengthMode::HoldAfterTarget;
    throw std::invalid_argument("Unsupported length mode: " + input);
}

const char* to_string(DeviceKind kind) {
    switch (kind) {
        case DeviceKind::SonoControlFpga: return "sonocontrol_fpga";
        case DeviceKind::Hyus: return "hyus";
    }
    return "sonocontrol_fpga";
}

DeviceKind device_kind_from_string(const std::string& input) {
    const auto s = lower_copy(input);
    if (s == "sonocontrol_fpga" || s == "sonocontrol" || s == "fpga" || s == "legacy") return DeviceKind::SonoControlFpga;
    if (s == "hyus" || s == "lan" || s == "tcp") return DeviceKind::Hyus;
    throw std::invalid_argument("Unsupported device kind: " + input);
}

} // namespace sonocontrol

namespace sonocontrol::protocol {

namespace {

ComPacket com_packet(uint8_t cmd, uint32_t payload_le_value) {
    ComPacket p{};
    p[0] = 0xAA;
    p[1] = 0x03;
    p[2] = cmd;
    p[3] = static_cast<uint8_t>(payload_le_value & 0xFFu);
    p[4] = static_cast<uint8_t>((payload_le_value >> 8u) & 0xFFu);
    p[5] = static_cast<uint8_t>((payload_le_value >> 16u) & 0xFFu);
    p[6] = static_cast<uint8_t>((payload_le_value >> 24u) & 0xFFu);
    p[7] = 0x88;
    return p;
}

AmplitudeTable sine_table(double amplitude, double duty_cycle) {
    AmplitudeTable table{};
    table.fill(MIDPOINT);
    const auto duty_count = static_cast<int>(std::llround(duty_cycle * static_cast<double>(SAMPLE_COUNT)));
    if (duty_count < 2) return table;

    const double n = static_cast<double>(duty_count - 1);
    const double k = amplitude * static_cast<double>(MIDPOINT);
    constexpr double PI = 3.141592653589793238462643383279502884;
    for (int i = 0; i < static_cast<int>(SAMPLE_COUNT); ++i) {
        if (i < duty_count) {
            const int raw = static_cast<int>(MIDPOINT) + static_cast<int>(k * std::sin(PI * static_cast<double>(i) / n));
            table[static_cast<size_t>(i)] = clamp_raw(raw);
        }
    }
    return table;
}

AmplitudeTable square_table(double amplitude, double duty_cycle) {
    AmplitudeTable table{};
    table.fill(MIDPOINT);
    const auto duty_count = static_cast<int>(std::llround(duty_cycle * static_cast<double>(SAMPLE_COUNT)));
    const auto hi = clamp_raw(static_cast<int>(MIDPOINT) + static_cast<int>(amplitude * static_cast<double>(MIDPOINT)));
    for (int i = 0; i < duty_count && i < static_cast<int>(SAMPLE_COUNT); ++i) {
        table[static_cast<size_t>(i)] = hi;
    }
    return table;
}

AmplitudeTable triangle_table(double amplitude, double duty_cycle) {
    AmplitudeTable table{};
    table.fill(MIDPOINT);
    const auto duty_count = static_cast<int>(std::llround(duty_cycle * static_cast<double>(SAMPLE_COUNT)));
    if (duty_count < 2) return table;

    const int half = duty_count / 2;
    for (int i = 0; i < duty_count && i < static_cast<int>(SAMPLE_COUNT); ++i) {
        double a = 0.0;
        if (i < half) {
            a = amplitude * static_cast<double>(i) / static_cast<double>(std::max(1, half));
        } else {
            a = amplitude * static_cast<double>(duty_count - i) / static_cast<double>(std::max(1, duty_count - half));
        }
        table[static_cast<size_t>(i)] = clamp_raw(static_cast<int>(MIDPOINT) + static_cast<int>(a * static_cast<double>(MIDPOINT)));
    }
    return table;
}

} // namespace

uint32_t dds_word(double freq_hz) {
    if (freq_hz <= 0.0) return 0u;
    const double v = freq_hz * 4294967296.0 / static_cast<double>(CLOCK);
    if (v >= 4294967295.0) return 0xFFFFFFFFu;
    return static_cast<uint32_t>(v); // floor truncation, same as Python int()
}

ComPacket pkt_prf(double prf_hz) {
    return com_packet(0x10, dds_word(prf_hz));
}

ComPacket pkt_cfreq(double freq_hz) {
    return com_packet(0x11, dds_word(freq_hz));
}

ComPacket pkt_duration(int ms) {
    return com_packet(0x06, static_cast<uint32_t>(std::max(0, ms)));
}

ComPacket pkt_stop() {
    return pkt_duration(0);
}

uint16_t clamp_raw(int raw) {
    return static_cast<uint16_t>(std::max(0, std::min(static_cast<int>(MAX_RAW), raw)));
}

std::array<uint8_t, 8> udp_sample(uint16_t index, uint16_t raw_amp) {
    const auto raw = clamp_raw(raw_amp);
    return {0xBB, 0x03, 0x00,
            static_cast<uint8_t>((index >> 8u) & 0xFFu), static_cast<uint8_t>(index & 0xFFu),
            static_cast<uint8_t>((raw >> 8u) & 0xFFu), static_cast<uint8_t>(raw & 0xFFu),
            0x88};
}

AmplitudeTable build_amplitude_table(double amplitude, double duty_cycle, WaveShape shape) {
    amplitude = std::clamp(amplitude, 0.0, 1.0);
    duty_cycle = std::clamp(duty_cycle, 0.0, 1.0);
    switch (shape) {
        case WaveShape::Sine: return sine_table(amplitude, duty_cycle);
        case WaveShape::Square: return square_table(amplitude, duty_cycle);
        case WaveShape::Triangle: return triangle_table(amplitude, duty_cycle);
    }
    return sine_table(amplitude, duty_cycle);
}

std::vector<uint8_t> build_udp_burst(double amplitude, double duty_cycle, WaveShape shape) {
    const auto table = build_amplitude_table(amplitude, duty_cycle, shape);
    std::vector<uint8_t> burst(SAMPLE_COUNT * 8);
    uint8_t* p = burst.data();
    for (uint16_t i = 0; i < SAMPLE_COUNT; ++i, p += 8) {
        const uint16_t raw = clamp_raw(table[i]);
        p[0] = 0xBB;
        p[1] = 0x03;
        p[2] = 0x00;
        p[3] = static_cast<uint8_t>((i >> 8) & 0xFF);
        p[4] = static_cast<uint8_t>(i & 0xFF);
        p[5] = static_cast<uint8_t>((raw >> 8) & 0xFF);
        p[6] = static_cast<uint8_t>(raw & 0xFF);
        p[7] = 0x88;
    }
    return burst;
}

std::vector<float> waveform_float_array(double amplitude, double duty_cycle, WaveShape shape) {
    const auto table = build_amplitude_table(amplitude, duty_cycle, shape);
    std::vector<float> out;
    out.reserve(SAMPLE_COUNT);
    for (auto v : table) {
        out.push_back(static_cast<float>((static_cast<double>(v) - static_cast<double>(MIDPOINT)) / static_cast<double>(MIDPOINT)));
    }
    return out;
}

std::string format_hex(const uint8_t* data, size_t size) {
    std::ostringstream os;
    os << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < size; ++i) {
        if (i) os << ' ';
        os << std::setw(2) << static_cast<int>(data[i]);
    }
    return os.str();
}

std::string format_hex(const ComPacket& packet) {
    return format_hex(packet.data(), packet.size());
}

} // namespace sonocontrol::protocol
