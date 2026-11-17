#include "hid_utility.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>

#ifdef __linux__
#include <filesystem>
#include <format>
#include <fstream>
#include <system_error>
#elif defined(_WIN32)
#include <algorithm>
#include <format>
#include <vector>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace headsetcontrol {

std::optional<std::string> get_hid_path(uint16_t vid, uint16_t pid, int iid, uint16_t usagepageid, uint16_t usageid)
{
    struct hid_device_info* devs = hid_enumerate(vid, pid);

    if (!devs) {
        fprintf(stderr, "HID enumeration failure.\n");
        return std::nullopt;
    }

    std::optional<std::string> result;

    // Because of a MacOS Bug beginning with Ventura 13.3, we ignore the interfaceid
    //   See https://github.com/Sapd/HeadsetControl/issues/281
#ifdef __APPLE__
    iid = 0;
#endif

    // usageid is more specific to interface id, so we try it first.
    // Older hidapi versions on non-Windows did not populate usage_page/usage, so
    // we only trust the match when both values are actually present; otherwise
    // we fall through to the interface_number fallback below.
    if (usageid && usagepageid) // ignore when one of them 0
    {
        struct hid_device_info* cur_dev = devs;
        while (cur_dev) {
            if (cur_dev->usage_page == usagepageid && cur_dev->usage == usageid) {
                result = std::string(cur_dev->path);
                break;
            }

            cur_dev = cur_dev->next;
        }
    }

    if (!result) // only when we didn't yet find something
    {
        struct hid_device_info* cur_dev = devs;
        while (cur_dev) {
            if (!iid || cur_dev->interface_number == iid) {
                result = std::string(cur_dev->path);
                break;
            }

            cur_dev = cur_dev->next;
        }
    }

    hid_free_enumeration(devs);

    return result;
}

std::optional<std::string> get_serial_path([[maybe_unused]] uint16_t vid, [[maybe_unused]] uint16_t pid)
{
#ifdef __linux__
    namespace fs = std::filesystem;

    auto read_attribute = [](const fs::path& path) {
        std::ifstream file(path);
        std::string value;
        file >> value;
        return value;
    };

    const auto vendor  = std::format("{:04x}", vid);
    const auto product = std::format("{:04x}", pid);

    // Each tty links to its USB interface, whose parent is the USB device
    std::error_code error;
    for (const auto& entry : fs::directory_iterator("/sys/class/tty", error)) {
        auto interface = fs::canonical(entry.path() / "device", error);
        if (error) {
            error.clear();
            continue;
        }
        if (read_attribute(interface.parent_path() / "idVendor") == vendor
            && read_attribute(interface.parent_path() / "idProduct") == product) {
            return "/dev/" + entry.path().filename().string();
        }
    }
#elif defined(_WIN32)
    std::vector<std::string> active_ports;
    HKEY serialcomm_key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &serialcomm_key) == ERROR_SUCCESS) {
        for (DWORD idx = 0;; ++idx) {
            char val_name[256] {};
            DWORD val_name_len = sizeof(val_name);
            DWORD type         = 0;
            char data[256] {};
            DWORD data_len   = sizeof(data) - 1;
            const LSTATUS st = RegEnumValueA(serialcomm_key, idx, val_name, &val_name_len, nullptr, &type,
                reinterpret_cast<LPBYTE>(data), &data_len);
            if (st != ERROR_SUCCESS) {
                break;
            }
            if (type == REG_SZ && data[0] != '\0') {
                active_ports.emplace_back(data);
            }
        }
        RegCloseKey(serialcomm_key);
    }

    if (active_ports.empty()) {
        return std::nullopt;
    }

    HKEY usb_key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SYSTEM\\CurrentControlSet\\Enum\\USB", 0, KEY_READ, &usb_key) != ERROR_SUCCESS) {
        return std::nullopt;
    }

    const auto prefix = std::format("VID_{:04X}&PID_{:04X}", vid, pid);
    std::optional<std::string> found_port;

    for (DWORD dev_idx = 0; !found_port; ++dev_idx) {
        char dev_name[256] {};
        DWORD dev_name_len = sizeof(dev_name);
        if (RegEnumKeyExA(usb_key, dev_idx, dev_name, &dev_name_len, nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS) {
            break;
        }
        if (_strnicmp(dev_name, prefix.c_str(), prefix.size()) != 0) {
            continue;
        }

        HKEY dev_key = nullptr;
        if (RegOpenKeyExA(usb_key, dev_name, 0, KEY_READ, &dev_key) != ERROR_SUCCESS) {
            continue;
        }

        for (DWORD inst_idx = 0; !found_port; ++inst_idx) {
            char inst_name[256] {};
            DWORD inst_name_len = sizeof(inst_name);
            if (RegEnumKeyExA(dev_key, inst_idx, inst_name, &inst_name_len, nullptr, nullptr, nullptr, nullptr) != ERROR_SUCCESS) {
                break;
            }

            const auto params_subkey = std::format("{}\\Device Parameters", inst_name);
            HKEY params_key          = nullptr;
            if (RegOpenKeyExA(dev_key, params_subkey.c_str(), 0, KEY_READ, &params_key) != ERROR_SUCCESS) {
                continue;
            }

            char port_name[64] {};
            DWORD port_name_len = sizeof(port_name) - 1;
            DWORD type          = 0;
            if (RegQueryValueExA(params_key, "PortName", nullptr, &type, reinterpret_cast<LPBYTE>(port_name), &port_name_len) == ERROR_SUCCESS
                && type == REG_SZ && port_name[0] != '\0') {
                const bool is_active = std::any_of(active_ports.begin(), active_ports.end(),
                    [&](const std::string& active) { return _stricmp(active.c_str(), port_name) == 0; });
                if (is_active) {
                    found_port = std::format("\\\\.\\{}", port_name);
                }
            }
            RegCloseKey(params_key);
        }

        RegCloseKey(dev_key);
    }

    RegCloseKey(usb_key);
    if (found_port) {
        return found_port;
    }
#endif

    return std::nullopt;
}

void close_hid_device(hid_device* handle)
{
    if (handle) {
        hid_close(handle);
    }
}

} // namespace headsetcontrol
