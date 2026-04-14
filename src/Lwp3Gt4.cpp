/**
 * @file Lwp3Gt4.cpp
 * @brief Implementation of the PorscheGt4 C++20 SDK.
 */

#include "Lwp3Gt4.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

#include "Lwp3Constants.hpp"

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {}
PorscheGt4::~PorscheGt4() {
    disconnect();
}

/**
 * @brief Scans for the specific peripheral using C++20 ranges.
 */
bool PorscheGt4::connect(std::string_view address) {
    while (true) {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        auto adapter = adapters[0];
        std::cout << "\r[SDK] Searching for Porsche at " << address << "... " << std::flush;

        adapter.scan_for(1000);
        auto results = adapter.scan_get_results();

        // Range-based search with non-const lambda to satisfy SimpleBLE's API
        auto it = std::ranges::find_if(results, [&](auto& p) { return p.address() == address; });

        if (it != results.end()) {
            porsche = *it;
            std::cout << "\n[SDK] Hub detected! Connecting..." << std::endl;
            try {
                porsche.connect();
                std::this_thread::sleep_for(1500ms);
                setupNotifications();

                // Enable Telemetry updates
                auto enable_prop = [&](HubProperty prop) {
                    sendRaw({static_cast<uint8_t>(MessageLength::HUB_PROP_SETUP), 0x00,
                             static_cast<uint8_t>(MessageType::HUB_PROPERTIES),
                             static_cast<uint8_t>(prop),
                             static_cast<uint8_t>(HubPropertyOperation::ENABLE_UPDATES)});
                };

                enable_prop(HubProperty::BATTERY_VOLTAGE);
                enable_prop(HubProperty::BUTTON_STATE);
                enable_prop(HubProperty::RSSI);

                return true;
            } catch (const std::exception& e) {
                std::cerr << "\n[SDK] Handshake error: " << e.what() << ". Retrying..."
                          << std::endl;
            }
        }
        std::this_thread::sleep_for(500ms);
    }
}

/**
 * @brief Processes incoming LWP3 notifications. Uses std::memcpy for aliasing safety.
 */
void PorscheGt4::setupNotifications() {
    try {
        porsche.notify(SERVICE_UUID.data(), CHAR_UUID.data(), [this](SimpleBLE::ByteArray data) {
            const auto* raw = reinterpret_cast<const uint8_t*>(data.data());
            if (data.size() < 3) return;

            const auto msgType = static_cast<MessageType>(raw[2]);

            switch (msgType) {
                case MessageType::PORT_VALUE_SINGLE:
                    if (raw[3] == PORT_STEER) {
                        int32_t pos = 0;
                        // Use std::memcpy to avoid Strict Aliasing UB and Alignment issues
                        std::memcpy(&pos, &raw[4], sizeof(int32_t));
                        rawSteerPos.store(pos);
                        telemetryActive.store(true);

                        // Wake up blocking threads (calibration/homing)
                        cv_movement.notify_all();

                        if (onSteerUpdate) onSteerUpdate(pos - hardwareCenter);
                    }
                    break;

                case MessageType::HUB_PROPERTIES: {
                    const auto prop = static_cast<HubProperty>(raw[3]);
                    if (static_cast<HubPropertyOperation>(raw[4]) != HubPropertyOperation::UPDATE)
                        break;

                    if (prop == HubProperty::BATTERY_VOLTAGE) {
                        batteryLevel.store(raw[5]);
                        if (onBatteryUpdate) onBatteryUpdate(raw[5]);
                    } else if (prop == HubProperty::BUTTON_STATE) {
                        bool pressed = (raw[5] == 0x01);
                        buttonPressed.store(pressed);
                        if (onButtonUpdate) onButtonUpdate(pressed);
                    } else if (prop == HubProperty::RSSI) {
                        int8_t rssi = static_cast<int8_t>(raw[5]);
                        rssiValue.store(rssi);
                        if (onRssiUpdate) onRssiUpdate(rssi);
                    }
                    break;
                }
                default:
                    break;
            }
        });

        // Subscription for steering motor telemetry
        sendRaw({static_cast<uint8_t>(MessageLength::PORT_INPUT_FORMAT), 0x00,
                 static_cast<uint8_t>(MessageType::PORT_INPUT_FORMAT_SETUP), PORT_STEER, 0x02, 0x01,
                 0x00, 0x00, 0x00, 0x01});
    } catch (const std::exception& e) {
        std::cerr << "[SDK] Notification link error: " << e.what() << std::endl;
    }
}

/**
 * @brief Implementation of drive logic.
 * Corrects mirrored motor mounting by inverting the Left motor signal.
 */
void PorscheGt4::setDrive(int8_t speed) {
    setDrive(speed, speed);
}

/**
 * @brief Individual motor control. Left motor is inverted to compensate for chassis design.
 */
void PorscheGt4::setDrive(int8_t leftSpeed, int8_t rightSpeed) {
    auto drive = [&](uint8_t port, int8_t s, bool invert) {
        int final_speed = std::clamp(static_cast<int>(s), -100, 100);
        if (invert) final_speed *= -1;  // Compensate for physical orientation

        uint8_t val = static_cast<uint8_t>(final_speed);
        sendRaw({static_cast<uint8_t>(MessageLength::MOTOR_POWER), 0x00,
                 static_cast<uint8_t>(MessageType::PORT_OUTPUT_COMMAND), port, 0x11,
                 static_cast<uint8_t>(PortOutputSubCommand::START_POWER), val, 0x64, 0x03});
    };

    // Left motor (Port 0x32) is physically flipped compared to the Right motor (Port 0x33)
    drive(PORT_DRIVE_L, leftSpeed, true);
    drive(PORT_DRIVE_R, rightSpeed, false);
}

/**
 * @brief High-frequency steer command using stack-allocated std::array.
 */
void PorscheGt4::setSteerRaw(int32_t absolute_angle, uint8_t speed) {
    std::array<uint8_t, 14> buffer = {static_cast<uint8_t>(MessageLength::GOTO_ABS_POS),
                                      0x00,
                                      static_cast<uint8_t>(MessageType::PORT_OUTPUT_COMMAND),
                                      PORT_STEER,
                                      0x11,
                                      static_cast<uint8_t>(PortOutputSubCommand::GOTO_ABS_POS)};

    std::memcpy(&buffer[6], &absolute_angle, sizeof(int32_t));

    buffer[10] = speed;
    buffer[11] = 0x64;  // Max Power
    buffer[12] = 0x7E;  // Braking mode
    buffer[13] = 0x03;  // Complete feedback

    sendRaw(SimpleBLE::ByteArray(reinterpret_cast<const char*>(buffer.data()), buffer.size()));
}

/**
 * @brief Mechanical homing. Ensures valid range and maps center to 0.
 */
bool PorscheGt4::autoCalibrate() {
    if (!telemetryActive.load()) {
        for (int i = 0; i < 30 && !telemetryActive.load(); i++) std::this_thread::sleep_for(100ms);
    }
    if (!telemetryActive.load()) return false;

    const int32_t initialRawPos = rawSteerPos.load();
    hardwareCenter = initialRawPos;

    const int32_t rawLeft = sweep_to_limit(-30, 40, "LEFT");
    std::this_thread::sleep_for(500ms);

    setSteerRaw(initialRawPos, 50);
    waitForMovement(initialRawPos, 3, 4000);
    std::this_thread::sleep_for(500ms);

    const int32_t rawRight = sweep_to_limit(30, 40, "RIGHT");

    const int32_t totalTravel = std::abs(rawLeft - rawRight);
    if (totalTravel < 80) return false;

    hardwareCenter = (rawLeft + rawRight) / 2;
    minRelative.store(std::min(rawLeft, rawRight) - hardwareCenter);
    maxRelative.store(std::max(rawLeft, rawRight) - hardwareCenter);

    setSteer(0);
    waitForMovement(hardwareCenter, 1, 3000);
    return true;
}

/**
 * @brief Blocks using Condition Variable until target position is reached.
 */
void PorscheGt4::waitForMovement(int32_t target_absolute, int32_t tolerance, int timeout_ms) {
    std::unique_lock<std::mutex> lock(mtx_movement);
    cv_movement.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
        return std::abs(rawSteerPos.load() - target_absolute) <= tolerance;
    });
}

/**
 * @brief Low-level power sweep to find mechanical stalls.
 */
int32_t PorscheGt4::sweep_to_limit(int8_t speed, uint8_t power, std::string_view label) {
    sendRaw({static_cast<uint8_t>(MessageLength::MOTOR_POWER), 0x00,
             static_cast<uint8_t>(MessageType::PORT_OUTPUT_COMMAND), PORT_STEER, 0x11,
             static_cast<uint8_t>(PortOutputSubCommand::START_POWER), static_cast<uint8_t>(speed),
             power, 0x03});

    std::this_thread::sleep_for(600ms);

    int32_t last_pos = rawSteerPos.load();
    int stuck_count = 0;

    for (int i = 0; i < 80; i++) {
        std::this_thread::sleep_for(100ms);
        int32_t p = rawSteerPos.load();
        if (std::abs(p - last_pos) < 1)
            stuck_count++;
        else
            stuck_count = 0;

        if (stuck_count >= 4) break;
        last_pos = p;
    }

    stop();
    return rawSteerPos.load();
}

void PorscheGt4::setSteer(int32_t relative_angle) {
    int32_t clamped = std::clamp(relative_angle, minRelative.load(), maxRelative.load());
    setSteerRaw(hardwareCenter + clamped, 50);
}

void PorscheGt4::stop() {
    setDrive(0, 0);
    sendRaw({static_cast<uint8_t>(MessageLength::MOTOR_POWER), 0x00,
             static_cast<uint8_t>(MessageType::PORT_OUTPUT_COMMAND), PORT_STEER, 0x11,
             static_cast<uint8_t>(PortOutputSubCommand::START_POWER), 0x00, 0x64, 0x03});
}

void PorscheGt4::sendRaw(const SimpleBLE::ByteArray& data) {
    if (!porsche.is_connected()) return;
    try {
        porsche.write_request(SERVICE_UUID.data(), CHAR_UUID.data(), data);
    } catch (...) {
    }
}

void PorscheGt4::disconnect() {
    if (porsche.is_connected()) {
        stop();
        std::this_thread::sleep_for(500ms);
        porsche.disconnect();
    }
}

bool PorscheGt4::isConnected() {
    return porsche.is_connected();
}

}  // namespace LWP3