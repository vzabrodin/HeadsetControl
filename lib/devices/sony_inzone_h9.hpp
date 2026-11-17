#pragma once

#include "protocols/sony_inzone_protocol.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <format>
#include <span>
#include <string_view>
#include <vector>

using namespace std::string_view_literals;

namespace headsetcontrol {

/**
 * @brief Sony INZONE H9 (WH-G900N) / INZONE H7 (WH-G700) wireless gaming headset
 *
 * Communicates via a 2.4 GHz USB transceiver (VID 0x054C, PID 0x0E53).
 *
 * Unlike the newer INZONE H5 and H9 II (which wrap the Sony vendor HCI protocol
 * inside HID collection 0xFF04), the first-generation INZONE H9/H7 transceiver
 * routes the same Sony vendor HCI packets (commands 0x01 0x00 0xFC ..., events
 * 0x04 0xFF ...) directly over its USB CDC ACM virtual serial port (interface 6)
 * at 460800 baud.
 */
class SonyINZONEH9 : public protocols::SonyINZONEProtocol {
public:
    static constexpr std::array<uint16_t, 1> PRODUCT_IDS { 0x0E53 };

    static constexpr uint32_t BAUD_RATE  = 460800;
    static constexpr auto SETTLE_TIME    = std::chrono::milliseconds(50);
    static constexpr auto RETRY_INTERVAL = std::chrono::milliseconds(500);
    static constexpr auto REPLY_TIMEOUT  = std::chrono::seconds(2);

    std::vector<uint16_t> getProductIds() const override
    {
        return { PRODUCT_IDS.begin(), PRODUCT_IDS.end() };
    }

    std::string_view getDeviceName() const override { return "Sony INZONE H9"sv; }

    constexpr uint8_t getSupportedPlatforms() const override
    {
        return PLATFORM_LINUX | PLATFORM_WINDOWS;
    }

    constexpr bool usesSerialPort() const override { return true; }

    constexpr int getCapabilities() const override
    {
        return B(CAP_BATTERY_STATUS) | B(CAP_CHATMIX_STATUS)
            | B(CAP_SIDETONE) | B(CAP_LIGHTS) | B(CAP_INACTIVE_TIME)
            | B(CAP_BT_WHEN_POWERED_ON)
            | B(CAP_ANC) | B(CAP_ANC_STARTUP_MODE) | B(CAP_ANC_BUTTON_MODES)
            | B(CAP_MICROPHONE_MUTE_STATUS);
    }

    // The HID handle is only opened because HeadsetControl discovers devices via
    // HID enumeration. Select the vendor-defined collection (0xFF00:0x0001 on
    // interface 3) rather than Consumer Control (0x000C:0x0001) which Windows
    // locks exclusively.
    constexpr capability_detail getCapabilityDetail([[maybe_unused]] enum capabilities cap) const override
    {
        return { .usagepage = 0xFF00, .usageid = 0x0001, .interface_id = 3 };
    }

    Result<BatteryResult> getBattery(hid_device* device_handle) override
    {
        return getSonyBattery(device_handle);
    }

    Result<ChatmixResult> getChatmix(hid_device* device_handle) override
    {
        return getSonyChatmix(device_handle);
    }

    Result<SidetoneResult> setSidetone(hid_device* device_handle, uint8_t level) override
    {
        return setSonySidetone(device_handle, level);
    }

    Result<LightsResult> setLights(hid_device* device_handle, bool on) override
    {
        return setSonyLights(device_handle, on);
    }

    Result<InactiveTimeResult> setInactiveTime(hid_device* device_handle, uint8_t minutes) override
    {
        return setSonyInactiveTime(device_handle, minutes, false);
    }

    Result<BluetoothWhenPoweredOnResult> setBluetoothWhenPoweredOn(hid_device* device_handle, bool enabled) override
    {
        return setSonyBluetoothWhenPoweredOn(device_handle, enabled);
    }

    Result<AncResult> setAnc(hid_device* device_handle, uint8_t mode) override
    {
        return setSonyAnc(device_handle, mode);
    }

    Result<AncStartupModeResult> setAncStartupMode(hid_device* device_handle, uint8_t mode) override
    {
        return setSonyAncStartupMode(device_handle, mode);
    }

    Result<AncButtonModesResult> setAncButtonModes(
        hid_device* device_handle, const AncButtonModes& modes) override
    {
        return setSonyAncButtonModes(device_handle, modes);
    }

    Result<MicMuteStatusResult> getMicMuteStatus(hid_device* device_handle) override
    {
        return getSonyMicMuteStatus(device_handle);
    }

protected:
    Result<ParsedEvent> exchange([[maybe_unused]] hid_device* device_handle, uint8_t address,
        uint8_t event_id, uint8_t event_type, std::span<const uint8_t> payload) override
    {
        if (payload.size() > MAX_PAYLOAD_SIZE) {
            return DeviceError::invalidParameter(
                std::format("Payload of {} bytes exceeds maximum of {}", payload.size(), MAX_PAYLOAD_SIZE));
        }

        const uint16_t pid = getMatchedProductId() != 0 ? getMatchedProductId() : PRODUCT_IDS[0];
        auto handle        = getSerialInterface().open(getVendorId(), pid, BAUD_RATE, SETTLE_TIME);
        if (!handle) {
            return handle.error();
        }

        struct SerialGuard {
            SerialInterface& serial;
            int fd;
            ~SerialGuard() { serial.close(fd); }
        } guard { getSerialInterface(), *handle };

        const uint16_t tid = nextTransactionId();
        const auto cmd     = buildHciCommand(address, event_id, event_type, tid, payload);
        if (auto wr = getSerialInterface().write(*handle, cmd); !wr) {
            return wr.error();
        }

        const uint8_t want_type = (event_type == ETYPE_SET) ? ETYPE_NTFY : ETYPE_RET;
        std::vector<uint8_t> received;
        const auto deadline = std::chrono::steady_clock::now() + REPLY_TIMEOUT;

        while (true) {
            size_t offset      = 0;
            bool need_more_raw = false;

            while (offset + 3 <= received.size()) {
                if (received[offset] != HCI_TYPE_EVENT || received[offset + 1] != SONY_EVENT_CODE) {
                    ++offset;
                    continue;
                }

                const uint8_t param_len = received[offset + 2];
                if (param_len < 9 || param_len > MAX_PAYLOAD_SIZE + 9) {
                    ++offset;
                    continue;
                }

                const size_t frame_len = 3 + static_cast<size_t>(param_len);
                if (offset + frame_len > received.size()) {
                    need_more_raw = true;
                    break;
                }

                auto parsed = parseHciEvent(std::span<const uint8_t>(received.data() + offset, frame_len));
                if (!parsed) {
                    ++offset;
                    continue;
                }

                offset += frame_len;
                const bool tid_ok = (want_type == ETYPE_RET) || (parsed->transaction_id == tid);
                if (parsed->event_id == event_id
                    && tid_ok
                    && (parsed->event_type == want_type
                        || parsed->event_type == ETYPE_NTFY_ACTIVE)) {
                    return *parsed;
                }
            }

            if (offset > 0) {
                received.erase(received.begin(), received.begin() + static_cast<std::ptrdiff_t>(offset));
            }
            (void)need_more_raw;

            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now());
            if (remaining.count() <= 0) {
                return DeviceError::timeout(
                    std::format("No response for event_id 0x{:02x} (TID {})", event_id, tid));
            }

            const auto wait_ms = static_cast<int>(std::min(remaining, RETRY_INTERVAL).count());
            std::array<uint8_t, 256> buffer {};
            auto count = getSerialInterface().readTimeout(*handle, buffer, wait_ms);
            if (!count) {
                if (count.error().code == DeviceError::Code::Timeout) {
                    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - std::chrono::steady_clock::now());
                    if (left.count() <= 0) {
                        return DeviceError::timeout(
                            std::format("No response for event_id 0x{:02x} (TID {})", event_id, tid));
                    }
                    if (auto wr = getSerialInterface().write(*handle, cmd); !wr) {
                        return wr.error();
                    }
                    continue;
                }
                return count.error();
            }
            received.insert(received.end(), buffer.begin(), buffer.begin() + *count);
        }
    }
};

} // namespace headsetcontrol
