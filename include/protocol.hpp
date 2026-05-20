#pragma once

#include "config.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace sonocontrol::protocol {

constexpr uint32_t CLOCK = 100000000u;
constexpr const char* UDP_HOST_DEFAULT = "192.168.0.2";
constexpr uint16_t UDP_PORT_DEFAULT = 5000;
constexpr size_t SAMPLE_COUNT = 4096;
constexpr uint16_t MIDPOINT = 0x1FFF;
constexpr uint16_t MAX_RAW = 0x3FFD;

using ComPacket = std::array<uint8_t, 8>;
using AmplitudeTable = std::array<uint16_t, SAMPLE_COUNT>;

uint32_t dds_word(double freq_hz);
ComPacket pkt_prf(double prf_hz);
ComPacket pkt_cfreq(double freq_hz);
ComPacket pkt_duration(int ms);
ComPacket pkt_stop();

uint16_t clamp_raw(int raw);
std::array<uint8_t, 8> udp_sample(uint16_t index, uint16_t raw_amp);
AmplitudeTable build_amplitude_table(double amplitude, double duty_cycle, WaveShape shape);
std::vector<uint8_t> build_udp_burst(double amplitude, double duty_cycle, WaveShape shape);
std::vector<float> waveform_float_array(double amplitude, double duty_cycle, WaveShape shape);
std::string format_hex(const uint8_t* data, size_t size);
std::string format_hex(const ComPacket& packet);

} // namespace sonocontrol::protocol
