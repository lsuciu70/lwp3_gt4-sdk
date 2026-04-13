/**
 * @file Lwp3Gt4.hpp
 * @author lsuciu
 * @brief Public Interface for the PorscheGt4 Controller.
 */

#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

namespace LWP3 {

/**
 * @class PorscheGt4
 * @brief High-level controller for the LEGO Technic Porsche GT4 (42176).
 */
class PorscheGt4 {
   public:
    // --- Callbacks ---

    /** @brief Callback for steering updates (Returns degrees relative to 0). */
    std::function<void(int32_t)> onSteerUpdate;

    /** @brief Callback for battery updates (Returns percentage 0-100). */
    std::function<void(uint8_t)> onBatteryUpdate;

    /** @brief Callback for physical Hub button events. */
    std::function<void(bool)> onButtonUpdate;

    /** @brief Callback for signal strength updates (Returns dBm). */
    std::function<void(int8_t)> onRssiUpdate;

    PorscheGt4();
    ~PorscheGt4();

    /**
     * @brief Scans and connects to the vehicle hub using a specific MAC address.
     * @param address Bluetooth MAC address.
     * @return True if connection and LWP3 handshake succeed.
     */
    bool connect(const std::string& address);

    /** @brief Disconnects and stops all hardware safely. */
    void disconnect();

    /**
     * @brief Performs mechanical homing to find steering limits.
     * @return True if calibration succeeded and a valid range was found.
     */
    bool autoCalibrate();

    /** @brief Sets steering position. Range determined by calibration. */
    void setSteer(int32_t relative_angle);

    /** @brief Sets drive speed using synchronized Virtual Port. */
    void setDrive(int8_t speed);

    /** @brief Sets individual drive speeds for differential maneuvers. */
    void setDrive(int8_t leftSpeed, int8_t rightSpeed);

    /** @brief Stops all drive and steering motors immediately. */
    void stop();

    // --- State Getters ---
    uint8_t getBatteryLevel() const;
    bool isButtonPressed() const;
    int32_t getMinSteer() const;
    int32_t getMaxSteer() const;
    bool isConnected();

   private:
    SimpleBLE::Peripheral porsche;

    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<uint8_t> batteryLevel{0};
    std::atomic<bool> buttonPressed{false};
    std::atomic<int8_t> rssiValue{0};
    std::atomic<bool> telemetryActive{false};
    std::atomic<uint8_t> virtualDrivePort{0xFF};

    int32_t hardwareCenter = 0;
    std::atomic<int32_t> minRelative{-60};
    std::atomic<int32_t> maxRelative{60};

    void setupNotifications();
    void sendRaw(SimpleBLE::ByteArray data);
    void setSteerRaw(int32_t absolute_angle, uint8_t speed = 40);
    int32_t sweep_to_limit(int8_t speed, uint8_t power, const std::string& label);
    void waitForMovement(int32_t target_absolute, int32_t tolerance = 2, int timeout_ms = 3000);
};
}  // namespace LWP3
