/**
 * @file Lwp3Gt4.hpp
 * @author lsuciu
 * @brief High-performance C++ SDK for the LEGO Technic Porsche GT4 (42176).
 * * This library implements the LEGO Wireless Protocol v3 (LWP3) to provide
 * precise control over the Porsche's steering and drive motors, including
 * automatic calibration and synchronized dual-motor traction.
 */

#pragma once
#include <simpleble/SimpleBLE.h>
#include <functional>
#include <cstdint>
#include <atomic>
#include <string>

namespace LWP3 {

    /** @brief LWP3 Packet Length Constants. */
    enum MessageLength : uint8_t {
        LEN_HUB_PROP_SETUP      = 0x05,
        LEN_VIRTUAL_PORT_SETUP  = 0x06,
        LEN_MOTOR_POWER         = 0x09,
        LEN_PORT_INPUT_FORMAT   = 0x0A,
        LEN_GOTO_ABS_POS        = 0x0E
    };

    /** @brief LWP3 Message Type Identifiers. */
    enum MessageType : uint8_t {
        HUB_PROPERTIES            = 0x01,
        HUB_ATTACHED_IO           = 0x04,
        GENERIC_ERROR_MESSAGES    = 0x05,
        PORT_INPUT_FORMAT_SETUP   = 0x41,
        PORT_VALUE_SINGLE         = 0x45,
        PORT_OUTPUT_COMMAND       = 0x81,
        VIRTUAL_PORT_SETUP        = 0x61
    };

    /** @brief Hub Property Identifiers. */
    enum HubProperty : uint8_t {
        BUTTON_STATE              = 0x02,
        RSSI                      = 0x05,
        BATTERY_VOLTAGE           = 0x06
    };

    /** @brief Operations for Hub Properties. */
    enum HubPropertyOperation : uint8_t {
        ENABLE_UPDATES            = 0x02,
        UPDATE                    = 0x06
    };

    /** @brief Sub-commands for Port Output. */
    enum PortOutputSubCommand : uint8_t {
        START_POWER               = 0x07,
        GOTO_ABS_POS              = 0x0D
    };

    /**
     * @class PorscheGt4
     * @brief Controller class for the Porsche GT4.
     */
    class PorscheGt4 {
    public:
        // --- Hardware Constants ---
        static constexpr uint8_t PORT_STEER   = 0x34; /**< Internal steering motor port. */
        static constexpr uint8_t PORT_DRIVE_L = 0x32; /**< Left rear traction motor. */
        static constexpr uint8_t PORT_DRIVE_R = 0x33; /**< Right rear traction motor. */
        
        static constexpr const char* SERVICE_UUID = "00001623-1212-efde-1623-785feabcd123";
        static constexpr const char* CHAR_UUID    = "00001624-1212-efde-1623-785feabcd123";

        // --- Callbacks ---
        
        /** @brief Triggered when steering position changes. Returns 0-centered degrees. */
        std::function<void(int32_t)> onSteerUpdate;
        
        /** @brief Triggered on battery level change (0-100%). */
        std::function<void(uint8_t)> onBatteryUpdate;
        
        /** @brief Triggered when the green Hub button is toggled. */
        std::function<void(bool)>    onButtonUpdate;
        
        /** @brief Triggered on signal strength updates (dBm). */
        std::function<void(int8_t)>  onRssiUpdate;

        PorscheGt4();
        ~PorscheGt4();

        /**
         * @brief Scans and connects to the Porsche Hub.
         * @param address The Bluetooth MAC address of the car.
         * @return True if connection and handshake succeed.
         */
        bool connect(const std::string& address);

        /** @brief Gracefully stops motors and closes the Bluetooth link. */
        void disconnect();

        /**
         * @brief Performs mechanical homing.
         * Sweeps steering to physical limits to calculate the true center.
         */
        void autoCalibrate();

        /**
         * @brief Set steering angle relative to center.
         * @param relative_angle Degrees from center (Negative=Left, Positive=Right).
         */
        void setSteer(int32_t relative_angle);

        /**
         * @brief Set speed for both rear motors simultaneously via Virtual Port.
         * @param speed Range -100 to 100.
         */
        void setDrive(int8_t speed); 

        /**
         * @brief Set individual speeds for rear motors (Differential Drive).
         * @param leftSpeed Left motor speed (-100 to 100).
         * @param rightSpeed Right motor speed (-100 to 100).
         */
        void setDrive(int8_t leftSpeed, int8_t rightSpeed);

        /** @brief Halts all motors immediately. */
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
        std::atomic<bool>    buttonPressed{false};
        std::atomic<int8_t>  rssiValue{0};
        std::atomic<bool>    telemetryActive{false};
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
}