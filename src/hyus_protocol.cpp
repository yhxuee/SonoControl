#include "hyus_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace sonocontrol::hyus {

namespace {
constexpr uint8_t SOH[4] = {0x53, 0x4F, 0x48, 0x20};

inline bool is_soh(const uint8_t* p) {
    return p[0] == SOH[0] && p[1] == SOH[1] && p[2] == SOH[2] && p[3] == SOH[3];
}

inline void put_u16le(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline void put_u32le(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
inline uint16_t get_u16le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t sum_bytes(const uint8_t* p, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return s;
}

uint32_t round_nonneg(double v) {
    if (v <= 0.0) return 0u;
    const double r = std::floor(v + 0.5);
    if (r >= 4294967295.0) return 0xFFFFFFFFu;
    return static_cast<uint32_t>(r);
}
} // namespace

uint16_t checksum_param(const uint8_t* first12) {
    return static_cast<uint16_t>((sum_bytes(first12, 12) + 0x0117u) & 0xFFFFu);
}
uint16_t checksum_ctrl15(const uint8_t* first13) {
    return static_cast<uint16_t>(sum_bytes(first13, 13) & 0xFFFFu);
}
uint16_t checksum_status(const uint8_t* first20) {
    return static_cast<uint16_t>(sum_bytes(first20, 20) & 0xFFFFu);
}

std::array<uint8_t, 14> make_param(Cmd cmd, uint32_t value) {
    std::array<uint8_t, 14> f{};
    std::memcpy(f.data(), SOH, 4);
    put_u16le(f.data() + 4, static_cast<uint16_t>(cmd));
    f[6] = 0x00;  // reserved
    f[7] = 0x00;
    put_u32le(f.data() + 8, value);
    put_u16le(f.data() + 12, checksum_param(f.data()));
    return f;
}

std::array<uint8_t, 26> write_param(Cmd cmd, uint32_t value) {
    std::array<uint8_t, 26> f{};
    std::memcpy(f.data(), PREFIX.data(), PREFIX.size());
    const auto param = make_param(cmd, value);
    std::memcpy(f.data() + PREFIX.size(), param.data(), param.size());
    return f;
}

void append_param(std::vector<uint8_t>& out, Cmd cmd, uint32_t value) {
    const auto f = write_param(cmd, value);
    out.insert(out.end(), f.begin(), f.end());
}

std::array<uint8_t, 15> make_ctrl15(uint16_t selector) {
    std::array<uint8_t, 15> f{};
    std::memcpy(f.data(), SOH, 4);
    f[4] = 0x01; f[5] = 0x00; f[6] = 0x00; f[7] = 0x00;
    put_u16le(f.data() + 8, selector);
    f[10] = 0x01; f[11] = 0x03; f[12] = 0x01;
    put_u16le(f.data() + 13, checksum_ctrl15(f.data()));
    return f;
}

uint32_t encode_cfreq(double freq_hz)         { return round_nonneg(freq_hz / 31250.0); }
uint32_t encode_amplitude_percent(double pct) { return round_nonneg(std::clamp(pct, 0.0, 100.0)); }
uint32_t encode_pulse_len_us(double us)       { return round_nonneg(us * 125.0); }
uint32_t encode_pulse_period_us(double us)    { return round_nonneg(us * 125.0); }
uint32_t encode_seq_len_ms(double ms)         { return round_nonneg(ms * 125000.0); }
uint32_t encode_seq_period_ms(double ms)      { return round_nonneg(ms * 125000.0); }
uint32_t encode_total_duration_s(double s)    { return round_nonneg(s); }

std::optional<StatusFrame> parse_status(const uint8_t* p, size_t n) {
    if (n < 22 || !is_soh(p)) return std::nullopt;
    // Discriminator for the 22-byte status family: 08 00 00 00.
    if (!(p[4] == 0x08 && p[5] == 0x00 && p[6] == 0x00 && p[7] == 0x00)) return std::nullopt;
    StatusFrame s;
    s.index = p[12];
    s.subtype = p[13];
    std::memcpy(s.data.data(), p + 14, 6);
    s.checksum_ok = (get_u16le(p + 20) == checksum_status(p));
    return s;
}

size_t frame_length_at(const uint8_t* p, size_t available) {
    if (available < 4 || !is_soh(p)) return 0;
    if (available < 8) return 0;  // need the discriminator word
    // PREFIX: 53 4F 48 20 0C 00 00 00 ...
    if (p[4] == 0x0C && p[5] == 0x00 && p[6] == 0x00 && p[7] == 0x00) return 12;
    // STATUS: 53 4F 48 20 08 00 00 00 ...
    if (p[4] == 0x08 && p[5] == 0x00 && p[6] == 0x00 && p[7] == 0x00) return 22;
    // CTRL15: 53 4F 48 20 01 00 00 00 ...
    if (p[4] == 0x01 && p[5] == 0x00 && p[6] == 0x00 && p[7] == 0x00) return 15;
    // Otherwise assume a 14-byte PARAM frame (CMD in p[4..5]).
    return 14;
}

std::vector<std::vector<uint8_t>> FrameReassembler::feed(const uint8_t* data, size_t size) {
    buf_.insert(buf_.end(), data, data + size);
    std::vector<std::vector<uint8_t>> frames;
    size_t pos = 0;
    while (pos + 4 <= buf_.size()) {
        const uint8_t* p = buf_.data() + pos;
        if (!is_soh(p)) { ++pos; continue; }  // resync to the next SOH marker
        const size_t avail = buf_.size() - pos;
        const size_t len = frame_length_at(p, avail);
        if (len == 0 || len > avail) break;    // need more bytes
        frames.emplace_back(p, p + len);
        pos += len;
    }
    if (pos > 0) buf_.erase(buf_.begin(), buf_.begin() + pos);
    return frames;
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

} // namespace sonocontrol::hyus
