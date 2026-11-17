#include "serial_interface.hpp"

#include "../hid_utility.hpp"

#include <format>
#include <optional>

#ifdef __linux__
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#elif defined(_WIN32)
#include <mutex>
#include <thread>
#include <unordered_map>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace headsetcontrol {

#ifdef __linux__

namespace {

    std::optional<speed_t> toSpeed(uint32_t baud_rate)
    {
        switch (baud_rate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            return std::nullopt;
        }
    }

} // namespace

auto RealSerialInterface::open(uint16_t vid, uint16_t pid, uint32_t baud_rate,
    std::chrono::milliseconds settle_time) -> Result<int>
{
    auto speed = toSpeed(baud_rate);
    if (!speed) {
        return DeviceError::invalidParameter(std::format("Unsupported baud rate {}", baud_rate));
    }

    auto path = get_serial_path(vid, pid);
    if (!path) {
        return DeviceError::deviceOffline("Serial port not found");
    }

    int fd = ::open(path->c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return DeviceError::hidError(
            std::format("Could not open {}, check its permissions (headsetcontrol -u prints udev rules)", *path));
    }

    termios settings {};
    if (tcgetattr(fd, &settings) != 0) {
        ::close(fd);
        return DeviceError::hidError("Could not read serial port settings");
    }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    cfsetspeed(&settings, *speed);
    if (tcsetattr(fd, TCSANOW, &settings) != 0) {
        ::close(fd);
        return DeviceError::hidError("Could not apply serial port settings");
    }

    std::this_thread::sleep_for(settle_time);
    tcflush(fd, TCIFLUSH);
    return fd;
}

void RealSerialInterface::close(int handle)
{
    if (handle >= 0) {
        ::close(handle);
    }
}

auto RealSerialInterface::write(int handle, std::span<const uint8_t> data) -> Result<void>
{
    const auto bytes_written = ::write(handle, data.data(), data.size());
    if (bytes_written != static_cast<ssize_t>(data.size())) [[unlikely]] {
        return DeviceError::hidError(
            std::format("Serial write failed: {} bytes, expected {}", bytes_written, data.size()));
    }
    return {};
}

auto RealSerialInterface::readTimeout(int handle, std::span<uint8_t> data, int timeout_ms) -> Result<size_t>
{
    pollfd waiting { .fd = handle, .events = POLLIN, .revents = 0 };
    const int ready = ::poll(&waiting, 1, timeout_ms);
    if (ready < 0) [[unlikely]] {
        return DeviceError::hidError("Could not wait for serial port");
    }
    if (ready == 0) {
        return DeviceError::timeout(std::format("Serial read timeout after {}ms (no data)", timeout_ms));
    }

    const auto bytes_read = ::read(handle, data.data(), data.size());
    if (bytes_read <= 0) [[unlikely]] {
        return DeviceError::hidError("Serial read failed");
    }
    return static_cast<size_t>(bytes_read);
}

#elif defined(_WIN32)

namespace {

    std::mutex g_serial_mutex;
    std::unordered_map<int, HANDLE> g_serial_handles;
    int g_next_serial_id = 1;

    std::optional<DWORD> toSpeed(uint32_t baud_rate)
    {
        switch (baud_rate) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 230400:
        case 460800:
        case 921600:
            return static_cast<DWORD>(baud_rate);
        default:
            return std::nullopt;
        }
    }

    HANDLE lookupHandle(int handle)
    {
        std::lock_guard<std::mutex> lock(g_serial_mutex);
        auto it = g_serial_handles.find(handle);
        return (it != g_serial_handles.end()) ? it->second : INVALID_HANDLE_VALUE;
    }

} // namespace

auto RealSerialInterface::open(uint16_t vid, uint16_t pid, uint32_t baud_rate,
    std::chrono::milliseconds settle_time) -> Result<int>
{
    auto speed = toSpeed(baud_rate);
    if (!speed) {
        return DeviceError::invalidParameter(std::format("Unsupported baud rate {}", baud_rate));
    }

    auto path = get_serial_path(vid, pid);
    if (!path) {
        return DeviceError::deviceOffline("Serial port not found");
    }

    HANDLE h = CreateFileA(path->c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        if (GetLastError() == ERROR_ACCESS_DENIED) {
            return DeviceError::hidError(
                std::format("Could not open {} (access denied — close vendor hub software if running)", *path));
        }
        return DeviceError::hidError(std::format("Could not open {}", *path));
    }

    DCB dcb {};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return DeviceError::hidError("Could not read serial port settings");
    }

    dcb.BaudRate        = *speed;
    dcb.ByteSize        = 8;
    dcb.Parity          = NOPARITY;
    dcb.StopBits        = ONESTOPBIT;
    dcb.fBinary         = TRUE;
    dcb.fParity         = FALSE;
    dcb.fOutxCtsFlow    = FALSE;
    dcb.fOutxDsrFlow    = FALSE;
    dcb.fDtrControl     = DTR_CONTROL_ENABLE;
    dcb.fRtsControl     = RTS_CONTROL_ENABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX           = FALSE;
    dcb.fInX            = FALSE;
    dcb.fErrorChar      = FALSE;
    dcb.fNull           = FALSE;
    dcb.fAbortOnError   = FALSE;

    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        return DeviceError::hidError("Could not apply serial port settings");
    }

    if (settle_time.count() > 0) {
        std::this_thread::sleep_for(settle_time);
    }
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    std::lock_guard<std::mutex> lock(g_serial_mutex);
    const int id         = g_next_serial_id++;
    g_serial_handles[id] = h;
    return id;
}

void RealSerialInterface::close(int handle)
{
    HANDLE h = INVALID_HANDLE_VALUE;
    {
        std::lock_guard<std::mutex> lock(g_serial_mutex);
        auto it = g_serial_handles.find(handle);
        if (it != g_serial_handles.end()) {
            h = it->second;
            g_serial_handles.erase(it);
        }
    }
    if (h != INVALID_HANDLE_VALUE) {
        CloseHandle(h);
    }
}

auto RealSerialInterface::write(int handle, std::span<const uint8_t> data) -> Result<void>
{
    HANDLE h = lookupHandle(handle);
    if (h == INVALID_HANDLE_VALUE) [[unlikely]] {
        return DeviceError::hidError("Invalid serial port handle");
    }

    DWORD bytes_written = 0;
    if (!WriteFile(h, data.data(), static_cast<DWORD>(data.size()), &bytes_written, nullptr)
        || bytes_written != static_cast<DWORD>(data.size())) [[unlikely]] {
        return DeviceError::hidError(
            std::format("Serial write failed: {} bytes, expected {}", bytes_written, data.size()));
    }
    return {};
}

auto RealSerialInterface::readTimeout(int handle, std::span<uint8_t> data, int timeout_ms) -> Result<size_t>
{
    HANDLE h = lookupHandle(handle);
    if (h == INVALID_HANDLE_VALUE) [[unlikely]] {
        return DeviceError::hidError("Invalid serial port handle");
    }

    COMMTIMEOUTS timeouts {};
    timeouts.ReadIntervalTimeout         = 20;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = static_cast<DWORD>(timeout_ms > 0 ? timeout_ms : 1);
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 1000;
    if (!SetCommTimeouts(h, &timeouts)) [[unlikely]] {
        return DeviceError::hidError("Could not set serial port timeouts");
    }

    DWORD bytes_read = 0;
    if (!ReadFile(h, data.data(), static_cast<DWORD>(data.size()), &bytes_read, nullptr)) [[unlikely]] {
        return DeviceError::hidError("Serial read failed");
    }
    if (bytes_read == 0) {
        return DeviceError::timeout(std::format("Serial read timeout after {}ms (no data)", timeout_ms));
    }
    return static_cast<size_t>(bytes_read);
}

#else

auto RealSerialInterface::open(uint16_t, uint16_t, uint32_t, std::chrono::milliseconds) -> Result<int>
{
    return DeviceError::notSupported("Serial ports are only supported on Linux and Windows for now");
}

void RealSerialInterface::close(int) { }

auto RealSerialInterface::write(int, std::span<const uint8_t>) -> Result<void>
{
    return DeviceError::notSupported("Serial ports are only supported on Linux and Windows for now");
}

auto RealSerialInterface::readTimeout(int, std::span<uint8_t>, int) -> Result<size_t>
{
    return DeviceError::notSupported("Serial ports are only supported on Linux and Windows for now");
}

#endif

} // namespace headsetcontrol
