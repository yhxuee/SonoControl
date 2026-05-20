#include "serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#ifndef _WIN32
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace sonocontrol {

SerialPort::~SerialPort() {
    close();
}

#ifdef _WIN32

namespace {
std::string normalize_windows_port(const std::string& port) {
    if (port.rfind("\\\\.\\", 0) == 0) return port;
    if (port.rfind("COM", 0) == 0 || port.rfind("com", 0) == 0) return "\\\\.\\" + port;
    return port;
}
}

bool SerialPort::open(const std::string& port, int baudrate, int bytesize, char parity, int stopbits, int timeout_ms) {
    close();
    const auto win_port = normalize_windows_port(port);
    handle_ = CreateFileA(win_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) return false;

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) {
        close();
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baudrate);
    dcb.ByteSize = static_cast<BYTE>(bytesize);
    dcb.Parity = (parity == 'E') ? EVENPARITY : (parity == 'O') ? ODDPARITY : NOPARITY;
    dcb.StopBits = (stopbits == 2) ? TWOSTOPBITS : ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = (parity != 'N');

    if (!SetCommState(handle_, &dcb)) {
        close();
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = static_cast<DWORD>(timeout_ms);
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(timeout_ms);
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = static_cast<DWORD>(timeout_ms);
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(handle_, &timeouts);
    PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
}

void SerialPort::close() {
    if (handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(handle_);
        handle_ = INVALID_HANDLE_VALUE;
    }
}

bool SerialPort::is_open() const {
    return handle_ != INVALID_HANDLE_VALUE;
}

bool SerialPort::write_all(const uint8_t* data, size_t size) {
    if (!is_open()) return false;
    DWORD written = 0;
    return WriteFile(handle_, data, static_cast<DWORD>(size), &written, nullptr) && written == size;
}

std::vector<uint8_t> SerialPort::read_exact(size_t size) {
    std::vector<uint8_t> out(size);
    DWORD total = 0;
    while (total < size) {
        DWORD got = 0;
        if (!ReadFile(handle_, out.data() + total, static_cast<DWORD>(size - total), &got, nullptr)) break;
        if (got == 0) break;
        total += got;
    }
    out.resize(total);
    return out;
}


std::vector<uint8_t> SerialPort::read_available(size_t max_size) {
    std::vector<uint8_t> out;
    if (!is_open() || max_size == 0) return out;
    DWORD errors = 0;
    COMSTAT stat{};
    if (!ClearCommError(handle_, &errors, &stat)) return out;
    const DWORD avail = stat.cbInQue;
    if (avail == 0) return out;
    const DWORD want = static_cast<DWORD>(std::min<size_t>(max_size, avail));
    out.resize(want);
    DWORD got = 0;
    if (!ReadFile(handle_, out.data(), want, &got, nullptr)) {
        out.clear();
        return out;
    }
    out.resize(got);
    return out;
}

void SerialPort::reset_input_buffer() {
    if (is_open()) PurgeComm(handle_, PURGE_RXCLEAR);
}

void SerialPort::cancel_io() {
    // CancelIoEx with lpOverlapped=NULL cancels all pending I/O on the handle
    // regardless of which thread issued it. Works for both overlapped and
    // synchronous WriteFile/ReadFile, which is what we use here. A blocked
    // call returns with ERROR_OPERATION_ABORTED so the worker thread can exit.
    if (handle_ != INVALID_HANDLE_VALUE) {
        ::CancelIoEx(handle_, nullptr);
    }
}

#else

namespace {
speed_t baud_constant(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B9600;
    }
}
}

bool SerialPort::open(const std::string& port, int baudrate, int bytesize, char parity, int stopbits, int timeout_ms) {
    close();
    timeout_ms_ = timeout_ms;
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }

    cfmakeraw(&tty);
    const auto baud = baud_constant(baudrate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    switch (bytesize) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        default: tty.c_cflag |= CS8; break;
    }

    if (parity == 'E') {
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
    } else if (parity == 'O') {
        tty.c_cflag |= PARENB;
        tty.c_cflag |= PARODD;
    } else {
        tty.c_cflag &= ~PARENB;
    }

    if (stopbits == 2) tty.c_cflag |= CSTOPB;
    else tty.c_cflag &= ~CSTOPB;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = static_cast<cc_t>(std::max(1, timeout_ms / 100));

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::is_open() const {
    return fd_ >= 0;
}

bool SerialPort::write_all(const uint8_t* data, size_t size) {
    if (!is_open()) return false;
    size_t written_total = 0;
    while (written_total < size) {
        const ssize_t n = ::write(fd_, data + written_total, size - written_total);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            return false;
        }
        written_total += static_cast<size_t>(n);
    }
    tcdrain(fd_);
    return true;
}

std::vector<uint8_t> SerialPort::read_exact(size_t size) {
    std::vector<uint8_t> out;
    out.reserve(size);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
    while (out.size() < size && std::chrono::steady_clock::now() < deadline) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 20000;
        const int rv = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (rv > 0 && FD_ISSET(fd_, &rfds)) {
            uint8_t buf[64];
            const ssize_t n = ::read(fd_, buf, std::min(sizeof(buf), size - out.size()));
            if (n > 0) out.insert(out.end(), buf, buf + n);
        }
    }
    return out;
}


std::vector<uint8_t> SerialPort::read_available(size_t max_size) {
    std::vector<uint8_t> out;
    if (!is_open() || max_size == 0) return out;
    int available = 0;
    if (ioctl(fd_, FIONREAD, &available) != 0 || available <= 0) return out;
    const size_t want = std::min<size_t>(max_size, static_cast<size_t>(available));
    out.resize(want);
    const ssize_t n = ::read(fd_, out.data(), want);
    if (n <= 0) {
        out.clear();
        return out;
    }
    out.resize(static_cast<size_t>(n));
    return out;
}

void SerialPort::reset_input_buffer() {
    if (is_open()) tcflush(fd_, TCIFLUSH);
}

void SerialPort::cancel_io() {
    // POSIX path uses select()-based read_exact with a 20 ms poll, so it is
    // already responsive to stop_flag_ checks at the caller. write_all uses
    // EAGAIN polling. The most reliable cross-thread interrupt is shutting
    // down the fd, but serial fds don't accept shutdown(); closing is the
    // standard pattern. We deliberately don't reset fd_ here — the owning
    // thread will observe the next read/write returning EBADF and exit
    // cleanly, then call close() which is idempotent.
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

#endif

} // namespace sonocontrol
