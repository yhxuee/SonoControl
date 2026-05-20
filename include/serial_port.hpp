#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace sonocontrol {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& port,
              int baudrate,
              int bytesize,
              char parity,
              int stopbits,
              int timeout_ms);
    void close();
    bool is_open() const;
    bool write_all(const uint8_t* data, size_t size);
    std::vector<uint8_t> read_exact(size_t size);
    // Non-blocking/short-time read of bytes already available in the driver buffer.
    // Used by framed devices where fixed-length reads can become desynchronised.
    std::vector<uint8_t> read_available(size_t max_size);
    void reset_input_buffer();

    // Cancel any pending synchronous WriteFile/ReadFile on this handle from
    // another thread. Safe to call at any time, including when the port is
    // closed. Used by emergency-stop / watchdog paths to unblock a worker
    // thread that is wedged inside a USB-serial driver call (some FTDI/CH340
    // revisions ignore COMMTIMEOUTS under USB bus stalls).
    void cancel_io();

private:
#ifdef _WIN32
    HANDLE handle_ = INVALID_HANDLE_VALUE;
#else
    int fd_ = -1;
    int timeout_ms_ = 500;
#endif
};

} // namespace sonocontrol
