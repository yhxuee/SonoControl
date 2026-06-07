#pragma once

// Hyus (LAN / TCP) ultrasound device protocol.
//
// This module is intentionally self-contained: it depends on nothing from the
// existing FPGA protocol (protocol.hpp) so the new device can evolve and be
// unit-tested independently. It only builds/parses the application-layer
// `SOH ` frames documented in new_device_ref/PROTOCOL_DEV.md. Transport (the
// TCP server + UDP discovery beacon) lives elsewhere; everything here is pure
// byte manipulation with no sockets and no global state.
//
// All multi-byte integers are little-endian. Every frame begins with the
// 4-byte header "SOH " (0x53 0x4F 0x48 0x20). Three checksum families exist
// and were verified against 1346 captured frames with zero mismatches:
//   * 14-byte parameter frame : (sum(first 12) + 0x0117) & 0xFFFF
//   * 15-byte control/poll     : sum(first 13) & 0xFFFF
//   * 22-byte device status     : sum(first 20) & 0xFFFF

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sonocontrol::hyus {

// ---- Command / parameter IDs (PARAM frame, offset 4, little-endian u16) ----
enum class Cmd : uint16_t {
    ManualFire    = 0x1000,  // value=1, fire once (out of scope for v1 GUI)
    Cfreq         = 0x1001,  // value = freq_Hz / 31250
    Amplitude     = 0x1002,  // value = percent (0..100)
    PulseLen      = 0x1003,  // value = pulse_len_us  * 125
    PulsePeriod   = 0x1004,  // value = pulse_period_us * 125
    SeqLen        = 0x1005,  // value = seq_len_ms  * 125000
    SeqPeriod     = 0x1006,  // value = seq_period_ms * 125000
    TriggerSource = 0x1007,  // 0=internal, 1=external, 2=manual
    Run           = 0x1009,  // 0=stop/idle/clear, 1=start/arm/run
    TotalDuration = 0xFFFD,  // value = seconds
    RunMode       = 0xFFFE,  // 0=infinite repeat, 1=total-duration
};

enum class TriggerSource : uint32_t { Internal = 0, External = 1, Manual = 2 };
enum class RunMode : uint32_t { Repeat = 0, TotalDuration = 1 };

// The fixed 12-byte envelope that precedes every PARAM frame.
inline constexpr std::array<uint8_t, 12> PREFIX = {
    0x53, 0x4F, 0x48, 0x20, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};

// 15-byte control-frame selectors (offset 8).
inline constexpr uint16_t SELECTOR_POLL = 0xFFFF;  // ~1 Hz keepalive / query
inline constexpr uint16_t SELECTOR_INIT = 0xAAAA;  // leads a full parameter download

// UDP discovery beacon (PC -> broadcast :8193).
inline constexpr const char* DISCOVERY_PAYLOAD = "small animal ultrasound therapy device";

// ---- Checksums (exposed for testing / reuse) ----
uint16_t checksum_param(const uint8_t* first12);   // sum(12)+0x0117
uint16_t checksum_ctrl15(const uint8_t* first13);  // sum(13)
uint16_t checksum_status(const uint8_t* first20);  // sum(20)

// ---- Frame builders ----
// Bare 14-byte parameter frame (no PREFIX).
std::array<uint8_t, 14> make_param(Cmd cmd, uint32_t value);
// PREFIX + 14-byte parameter frame (26 bytes) — the normal on-wire unit.
std::array<uint8_t, 26> write_param(Cmd cmd, uint32_t value);
// 15-byte control frame (poll / init marker).
std::array<uint8_t, 15> make_ctrl15(uint16_t selector);
inline std::array<uint8_t, 15> make_poll() { return make_ctrl15(SELECTOR_POLL); }
inline std::array<uint8_t, 15> make_init() { return make_ctrl15(SELECTOR_INIT); }

// Append one PREFIX+PARAM group to a growing batch buffer.
void append_param(std::vector<uint8_t>& out, Cmd cmd, uint32_t value);

// ---- Value encoders (engineering units -> wire value) ----
uint32_t encode_cfreq(double freq_hz);           // /31250, rounded
uint32_t encode_amplitude_percent(double pct);   // clamped 0..100
uint32_t encode_pulse_len_us(double us);         // *125
uint32_t encode_pulse_period_us(double us);      // *125
uint32_t encode_seq_len_ms(double ms);           // *125000
uint32_t encode_seq_period_ms(double ms);        // *125000
uint32_t encode_total_duration_s(double s);

// ---- Parsed device status frame (22 bytes, DEV -> PC) ----
struct StatusFrame {
    uint8_t index = 0;                 // offset 12 (00/01/02 streaming; FD/FE on acks)
    uint8_t subtype = 0;               // offset 13 (0x20 streaming, 0x3F ack)
    std::array<uint8_t, 6> data{};     // offset 14..19
    bool checksum_ok = false;
};

// Parse a single 22-byte status frame starting at `p` (must point at "SOH ").
// Returns nullopt if it is not a well-formed status frame.
std::optional<StatusFrame> parse_status(const uint8_t* p, size_t n);

// ---- Stream reframer ----
// TCP is a byte stream; frames may be split or coalesced. Feed received bytes
// in; get back complete application frames split on the "SOH " marker and known
// per-type lengths. Leftover partial bytes are retained for the next feed().
class FrameReassembler {
public:
    // Each complete frame is returned as its own byte vector, in order.
    std::vector<std::vector<uint8_t>> feed(const uint8_t* data, size_t size);
    void reset() { buf_.clear(); }

private:
    std::vector<uint8_t> buf_;
};

// Length of a frame given its header bytes, or 0 if not yet determinable / unknown.
size_t frame_length_at(const uint8_t* p, size_t available);

std::string format_hex(const uint8_t* data, size_t size);

} // namespace sonocontrol::hyus
