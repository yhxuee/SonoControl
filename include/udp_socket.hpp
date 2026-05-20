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
};

} // namespace sonocontrol
