#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace sonocontrol {

class UdpSender {
public:
    UdpSender();
    ~UdpSender();

    UdpSender(const UdpSender&) = delete;
    UdpSender& operator=(const UdpSender&) = delete;

    bool open(uint16_t source_port);
    bool send_to(const uint8_t* data, size_t size, const std::string& host, uint16_t port);
    void close();

    // Force-cancel any pending sendto() from another thread. Used by
    // emergency-stop to unblock a worker that is wedged in a sendto loop
    // (rare, but possible when the kernel UDP buffer is full or routing
    // changes mid-send).
    void cancel_io();

private:
#ifdef _WIN32
    uintptr_t socket_ = static_cast<uintptr_t>(~0ull);
#else
    int socket_ = -1;
#endif

    // Cached destination resolved from the last (host, port) pair. The UDP
    // waveform burst sends 4096 packets to the same target in tight
    // succession, so caching skips 4095 redundant inet_pton/getaddrinfo
    // calls per burst (and avoids per-packet DNS when host[] is a hostname).
    // Stored as a raw network-order IPv4 to keep this header free of the
    // platform socket headers — sockaddr_in is rebuilt at send time.
    std::string cached_host_;
    uint32_t cached_addr_net_ = 0;
    uint16_t cached_port_ = 0;
    bool cached_dst_valid_ = false;
};

} // namespace sonocontrol
