#include "udp_socket.hpp"

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
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace sonocontrol {

#ifdef _WIN32
namespace {
struct WsaInit {
    WsaInit() { WSADATA wsa{}; WSAStartup(MAKEWORD(2, 2), &wsa); }
    ~WsaInit() { WSACleanup(); }
};
WsaInit& wsa_init() { static WsaInit init; return init; }
}
#endif

UdpSender::UdpSender() {
#ifdef _WIN32
    (void)wsa_init();
#endif
}

UdpSender::~UdpSender() {
    close();
}

bool UdpSender::open(uint16_t source_port) {
    close();
#ifdef _WIN32
    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) return false;
    BOOL reuse = TRUE;
    setsockopt(static_cast<SOCKET>(socket_), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(source_port);
    if (bind(static_cast<SOCKET>(socket_), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close();
        return false;
    }
#else
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) return false;
    int reuse = 1;
    setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(source_port);
    if (bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        close();
        return false;
    }
#endif
    return true;
}

bool UdpSender::send_to(const uint8_t* data, size_t size, const std::string& host, uint16_t port) {
#ifdef _WIN32
    if (socket_ == INVALID_SOCKET) return false;
#else
    if (socket_ < 0) return false;
#endif
    // Reuse the cached destination when (host, port) hasn't changed. Saves
    // 4095 inet_pton/getaddrinfo calls per UDP waveform burst and avoids
    // repeated DNS traffic if `host` happens to be a hostname.
    if (!cached_dst_valid_ || cached_port_ != port || cached_host_ != host) {
        in_addr ina{};
        if (inet_pton(AF_INET, host.c_str(), &ina) != 1) {
            addrinfo hints{};
            hints.ai_family = AF_INET;
            hints.ai_socktype = SOCK_DGRAM;
            addrinfo* res = nullptr;
            if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || res == nullptr) return false;
            ina = reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr;
            freeaddrinfo(res);
        }
        cached_addr_net_ = ina.s_addr;
        cached_host_ = host;
        cached_port_ = port;
        cached_dst_valid_ = true;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    dst.sin_addr.s_addr = cached_addr_net_;
#ifdef _WIN32
    const int sent = sendto(static_cast<SOCKET>(socket_), reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
                            reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    return sent == static_cast<int>(size);
#else
    const ssize_t sent = sendto(socket_, data, size, 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    return sent == static_cast<ssize_t>(size);
#endif
}

void UdpSender::close() {
#ifdef _WIN32
    if (socket_ != INVALID_SOCKET) {
        closesocket(static_cast<SOCKET>(socket_));
        socket_ = INVALID_SOCKET;
    }
#else
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
#endif
    cached_dst_valid_ = false;
    cached_host_.clear();
    cached_port_ = 0;
}

void UdpSender::cancel_io() {
#ifdef _WIN32
    if (socket_ != INVALID_SOCKET) {
        // CancelIoEx on a SOCKET wakes any sendto/recvfrom blocked on it.
        ::CancelIoEx(reinterpret_cast<HANDLE>(static_cast<SOCKET>(socket_)), nullptr);
    }
#else
    if (socket_ >= 0) {
        ::shutdown(socket_, SHUT_RDWR);
    }
#endif
}

} // namespace sonocontrol
