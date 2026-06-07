#pragma once

// Hyus LAN discovery + connection service.
//
// Protocol-faithful discovery (see new_device_ref/PROTOCOL_DEV.md): the PC
// broadcasts the discovery beacon on UDP `beaconPort` (~1 Hz) and listens for
// inbound TCP connections on `tcpPort`. The device dials in, so the PC is the
// TCP server. Each connected device is listed by its IP; the connection is held
// open (and a ~1 Hz poll frame is sent on it) so the entry stays "connected" and
// the same socket can be reused to send parameter/control frames.
//
// Self-contained: uses raw sockets (winsock on Windows) like udp_socket.cpp /
// serial_port.cpp, so it does not depend on the optional Qt Network module.
// Thread-safe: all public methods may be called from the GUI thread while the
// background discovery thread runs.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sonocontrol::hyus {

class HyusDiscovery {
public:
    HyusDiscovery() = default;
    ~HyusDiscovery();
    HyusDiscovery(const HyusDiscovery&) = delete;
    HyusDiscovery& operator=(const HyusDiscovery&) = delete;

    // Begins the background thread (beacon + TCP listener). Returns false if the
    // listen socket cannot be bound (port in use / firewall); lastError() has
    // the detail. Calling start() while already running is a no-op returning true.
    bool start(uint16_t tcpPort = 8192, uint16_t beaconPort = 8193);
    void stop();
    bool running() const { return running_.load(); }

    // Thread-safe snapshot of currently-connected device IPs.
    std::vector<std::string> devices() const;

    // Send raw bytes to a connected device. Returns false if the device is not
    // connected or the send failed.
    bool send(const std::string& ip, const uint8_t* data, size_t size);

    std::string lastError() const;

private:
    void run(uint16_t tcpPort, uint16_t beaconPort);
    void setError(const std::string& e);

    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> running_{false};

    // Guards connections_, devices_, and lastError_. The background thread and
    // the GUI thread (devices()/send()) both touch these.
    mutable std::mutex mtx_;
    struct Conn {
        std::uintptr_t sock;  // SOCKET / int, stored opaquely to keep the header socket-free
        std::string ip;
    };
    std::vector<Conn> connections_;
    std::vector<std::string> devices_;
    std::string lastError_;
};

} // namespace sonocontrol::hyus
