#include "hyus_net.hpp"

#include "hyus_protocol.hpp"

#include <chrono>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#endif

namespace sonocontrol::hyus {

namespace {
#ifdef _WIN32
using sock_t = SOCKET;
constexpr sock_t kBadSock = INVALID_SOCKET;
void close_sock(sock_t s) { if (s != kBadSock) closesocket(s); }
struct WsaInit {
    WsaInit() { WSADATA wsa{}; WSAStartup(MAKEWORD(2, 2), &wsa); }
    ~WsaInit() { WSACleanup(); }
};
void ensure_wsa() { static WsaInit init; (void)init; }
#else
using sock_t = int;
constexpr sock_t kBadSock = -1;
void close_sock(sock_t s) { if (s >= 0) ::close(s); }
void ensure_wsa() {}
#endif

std::string peer_ip(const sockaddr_in& addr) {
    char buf[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, const_cast<in_addr*>(&addr.sin_addr), buf, sizeof(buf));
    return std::string(buf);
}
} // namespace

HyusDiscovery::~HyusDiscovery() { stop(); }

void HyusDiscovery::setError(const std::string& e) {
    std::lock_guard<std::mutex> lk(mtx_);
    lastError_ = e;
}

std::string HyusDiscovery::lastError() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return lastError_;
}

std::vector<std::string> HyusDiscovery::devices() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return devices_;
}

bool HyusDiscovery::start(uint16_t tcpPort, uint16_t beaconPort) {
    if (running_.load()) return true;
    ensure_wsa();
    stop_.store(false);
    // Probe-bind the listen port up front so start() can report failure
    // synchronously (the common case is the port already in use or a firewall
    // block). The thread re-binds its own socket.
    {
        sock_t probe = ::socket(AF_INET, SOCK_STREAM, 0);
        if (probe == kBadSock) { setError("cannot create TCP socket"); return false; }
        int reuse = 1;
        setsockopt(probe, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(tcpPort);
        const int rc = bind(probe, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        close_sock(probe);
        if (rc != 0) {
            setError("cannot bind TCP port " + std::to_string(tcpPort) +
                     " (in use or blocked by firewall)");
            return false;
        }
    }
    running_.store(true);
    thread_ = std::thread([this, tcpPort, beaconPort] { run(tcpPort, beaconPort); });
    return true;
}

void HyusDiscovery::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
    running_.store(false);
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& c : connections_) close_sock(static_cast<sock_t>(c.sock));
    connections_.clear();
    devices_.clear();
}

bool HyusDiscovery::send(const std::string& ip, const uint8_t* data, size_t size) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& c : connections_) {
        if (c.ip != ip) continue;
        size_t off = 0;
        while (off < size) {
            const int n = ::send(static_cast<sock_t>(c.sock),
                                 reinterpret_cast<const char*>(data) + off,
                                 static_cast<int>(size - off), 0);
            if (n <= 0) return false;
            off += static_cast<size_t>(n);
        }
        return true;
    }
    return false;
}

void HyusDiscovery::run(uint16_t tcpPort, uint16_t beaconPort) {
    // UDP broadcast socket for the discovery beacon.
    sock_t udp = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp != kBadSock) {
        int yes = 1;
        setsockopt(udp, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&yes), sizeof(yes));
    }

    // TCP listen socket (the device dials in).
    sock_t listen_sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock == kBadSock) { setError("cannot create listen socket"); close_sock(udp); running_.store(false); return; }
    int reuse = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
    sockaddr_in laddr{};
    laddr.sin_family = AF_INET;
    laddr.sin_addr.s_addr = htonl(INADDR_ANY);
    laddr.sin_port = htons(tcpPort);
    if (bind(listen_sock, reinterpret_cast<sockaddr*>(&laddr), sizeof(laddr)) != 0 ||
        ::listen(listen_sock, 4) != 0) {
        setError("cannot bind/listen TCP port " + std::to_string(tcpPort));
        close_sock(listen_sock); close_sock(udp); running_.store(false); return;
    }

    const auto beacon = std::string(DISCOVERY_PAYLOAD);
    const auto poll = make_poll();
    auto last_beacon = std::chrono::steady_clock::now() - std::chrono::seconds(2);

    sockaddr_in baddr{};
    baddr.sin_family = AF_INET;
    baddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    baddr.sin_port = htons(beaconPort);

    while (!stop_.load()) {
        // ~1 Hz beacon + poll keepalive on all live connections.
        const auto now = std::chrono::steady_clock::now();
        if (now - last_beacon >= std::chrono::seconds(1)) {
            last_beacon = now;
            if (udp != kBadSock) {
                sendto(udp, beacon.data(), static_cast<int>(beacon.size()), 0,
                       reinterpret_cast<sockaddr*>(&baddr), sizeof(baddr));
            }
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto& c : connections_) {
                ::send(static_cast<sock_t>(c.sock), reinterpret_cast<const char*>(poll.data()),
                       static_cast<int>(poll.size()), 0);
            }
        }

        // Build the read set: listen socket + all live connections.
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(listen_sock, &rfds);
        sock_t maxfd = listen_sock;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto& c : connections_) {
                FD_SET(static_cast<sock_t>(c.sock), &rfds);
                if (static_cast<sock_t>(c.sock) > maxfd) maxfd = static_cast<sock_t>(c.sock);
            }
        }

        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 200000;  // 200 ms — also paces the 1 Hz beacon loop
#ifdef _WIN32
        const int sel = select(0, &rfds, nullptr, nullptr, &tv);
#else
        const int sel = select(static_cast<int>(maxfd) + 1, &rfds, nullptr, nullptr, &tv);
#endif
        if (sel < 0) continue;
        if (sel == 0) continue;

        // New device connection.
        if (FD_ISSET(listen_sock, &rfds)) {
            sockaddr_in caddr{};
#ifdef _WIN32
            int clen = sizeof(caddr);
#else
            socklen_t clen = sizeof(caddr);
#endif
            sock_t cs = accept(listen_sock, reinterpret_cast<sockaddr*>(&caddr), &clen);
            if (cs != kBadSock) {
                const std::string ip = peer_ip(caddr);
                std::lock_guard<std::mutex> lk(mtx_);
                // Replace any stale connection from the same IP.
                for (auto it = connections_.begin(); it != connections_.end();) {
                    if (it->ip == ip) { close_sock(static_cast<sock_t>(it->sock)); it = connections_.erase(it); }
                    else ++it;
                }
                connections_.push_back({static_cast<std::uintptr_t>(cs), ip});
                devices_.clear();
                for (auto& c : connections_) devices_.push_back(c.ip);
            }
        }

        // Drain readable connections; recv<=0 means the device disconnected.
        std::vector<sock_t> dropped;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            char buf[1024];
            for (auto& c : connections_) {
                const sock_t s = static_cast<sock_t>(c.sock);
                if (!FD_ISSET(s, &rfds)) continue;
                const int n = recv(s, buf, sizeof(buf), 0);
                if (n <= 0) dropped.push_back(s);
                // Status payload is currently all-zero telemetry; nothing to
                // decode yet (see PROTOCOL_DEV.md). The recv just keeps the
                // socket buffer drained and detects disconnects.
            }
            if (!dropped.empty()) {
                for (sock_t s : dropped) {
                    for (auto it = connections_.begin(); it != connections_.end();) {
                        if (static_cast<sock_t>(it->sock) == s) { close_sock(s); it = connections_.erase(it); }
                        else ++it;
                    }
                }
                devices_.clear();
                for (auto& c : connections_) devices_.push_back(c.ip);
            }
        }
    }

    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (auto& c : connections_) close_sock(static_cast<sock_t>(c.sock));
        connections_.clear();
        devices_.clear();
    }
    close_sock(listen_sock);
    close_sock(udp);
    running_.store(false);
}

} // namespace sonocontrol::hyus
