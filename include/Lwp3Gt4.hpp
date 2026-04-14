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
#include <string_view>

namespace LWP3 {

/**
 * @class PorscheGt4
 * @brief High-level controller for the LEGO Technic Porsche GT4 (42176).
 * * Implements thread-safe movement, automatic calibration, and
 * asynchronous telemetry using C++20 standards.
 */
class PorscheGt4 {
   public:
    // --- Asynchronous Callbacks ---

    /** @brief Triggered on steering change. Returns degrees relative to center. */
    std::function<void(int32_t)> onSteerUpdate;

    /** @brief Triggered on battery level change (0-100%). */
    std::function<void(uint8_t)> onBatteryUpdate;

    /** @brief Triggered when the physical Hub button is toggled. */
    std::function<void(bool)> onButtonUpdate;

    /** @brief Triggered on signal strength updates (dBm). */
    std::function<void(int8_t)> onRssiUpdate;

    PorscheGt4();
    ~PorscheGt4();

    /**
     * @brief Scans and connects to the Porsche Hub.
     * @param address Bluetooth MAC address (e.g., "28:3C:90:9C:82:14").
     * @return True if connection succeeds.
     */
    bool connect(std::string_view address);

    /** @brief Gracefully halts motors and closes the Bluetooth link. */
    void disconnect();

    /**
     * @brief Performs mechanical steering homing.
     * Sweeps to physical limits to determine true center and range.
     * @return True if a valid steering range was found.
     */
    bool autoCalibrate();

    /**
     * @brief Commands the steering motor to a specific normalized angle.
     * @param relative_angle Degrees from 0 (Negative=Left, Positive=Right).
     */
    void setSteer(int32_t relative_angle);

    /**
     * @brief Sets drive speed for the rear motors (Synchronized).
     * @note Internally handles motor inversion for symmetrical mounting.
     * @param speed Target speed from -100 (Reverse) to 100 (Forward).
     */
    void setDrive(int8_t speed);

    /**
     * @brief Sets individual speeds for rear motors (Differential Drive).
     * @param leftSpeed Left rear motor speed (-100 to 100).
     * @param rightSpeed Right rear motor speed (-100 to 100).
     */
    void setDrive(int8_t leftSpeed, int8_t rightSpeed);

    /** @brief Stops all motors immediately. */
    void stop();

    // --- State Getters ---
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

    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<uint8_t> batteryLevel{0};
    std::atomic<bool> buttonPressed{false};
    std::atomic<int8_t> rssiValue{0};
    std::atomic<bool> telemetryActive{false};

    std::mutex mtx_movement;
    std::condition_variable cv_movement;

    int32_t hardwareCenter = 0;
    std::atomic<int32_t> minRelative{-60};
    std::atomic<int32_t> maxRelative{60};

    void setupNotifications();
    void sendRaw(const SimpleBLE::ByteArray& data);
    void setSteerRaw(int32_t absolute_angle, uint8_t speed = 40);
    int32_t sweep_to_limit(int8_t speed, uint8_t power, std::string_view label);
    void waitForMovement(int32_t target_absolute, int32_t tolerance = 2, int timeout_ms = 3000);
};
}  // namespace LWP3