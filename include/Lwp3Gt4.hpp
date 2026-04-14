/**
 * @file Lwp3Gt4.hpp
 * @author lsuciu
 * @brief Public Interface for the PorscheGt4 C++20 SDK.
 */

#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>

namespace LWP3 {

/**
 * @class PorscheGt4
 * @brief Advanced controller for the LEGO Technic Porsche GT4 (42176).
 * Now updated to C++20 standards with improved safety and performance.
 */
class PorscheGt4 {
   public:
    // --- Callbacks ---
    std::function<void(int32_t)> onSteerUpdate;
    std::function<void(uint8_t)> onBatteryUpdate;
    std::function<void(bool)> onButtonUpdate;
    std::function<void(int8_t)> onRssiUpdate;

    PorscheGt4();
    ~PorscheGt4();

    /**
     * @brief Connects to the hub using C++20 string_view for zero-allocation literals.
     * @param address The MAC address of the Hub.
     * @return True if successful.
     */
    bool connect(std::string_view address);

    /** @brief Safely shuts down motors and disconnects. */
    void disconnect();

    /**
     * @brief Performs mechanical calibration.
     * @return True if a valid range was discovered.
     */
    bool autoCalibrate();

    /** @brief Sets steering relative to center. */
    void setSteer(int32_t relative_angle);

    /** @brief Sets drive speed for the synchronized Virtual Port. */
    void setDrive(int8_t speed);

    /** @brief Independent rear motor control. */
    void setDrive(int8_t leftSpeed, int8_t rightSpeed);

    /** @brief Immediate halt of all actuators. */
    void stop();

    // --- Getters ---
    uint8_t getBatteryLevel() const {
        return batteryLevel.load();
    }
    bool isButtonPressed() const {
        return buttonPressed.load();
    }
    int32_t getMinSteer() const {
        return minRelative.load();
    }
    int32_t getMaxSteer() const {
        return maxRelative.load();
    }
    bool isConnected();

   private:
    SimpleBLE::Peripheral porsche;

    // --- Thread-Safe State ---
    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<uint8_t> batteryLevel{0};
    std::atomic<bool> buttonPressed{false};
    std::atomic<int8_t> rssiValue{0};
    std::atomic<bool> telemetryActive{false};
    std::atomic<uint8_t> virtualDrivePort{0xFF};

    // --- Synchronization (C++20 Design) ---
    std::mutex mtx_movement;
    std::condition_variable cv_movement;

    int32_t hardwareCenter = 0;
    std::atomic<int32_t> minRelative{-60};
    std::atomic<int32_t> maxRelative{60};

    // --- Internal Protocol Logic ---
    void setupNotifications();

    /** @brief Sends raw data using const reference to avoid unnecessary copies. */
    void sendRaw(const SimpleBLE::ByteArray& data);

    /** @brief Move to position using stack-allocated buffers (no heap). */
    void setSteerRaw(int32_t absolute_angle, uint8_t speed = 40);

    int32_t sweep_to_limit(int8_t speed, uint8_t power, std::string_view label);

    /** @brief Smart wait using condition variables instead of busy-sleep. */
    void waitForMovement(int32_t target_absolute, int32_t tolerance = 2, int timeout_ms = 3000);
};
}  // namespace LWP3