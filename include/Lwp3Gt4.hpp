#pragma once
#include <simpleble/SimpleBLE.h>
#include <functional>
#include <vector>
#include <cstdint>
#include <iostream>
#include <atomic>

/**
 * @class Lwp3Gt4
 * @brief SDK for Porsche GT4 (42176).
 * Updated with blocking connection logic and continuous sweep calibration.
 */
class Lwp3Gt4 {
public:
    std::function<void(int32_t)> onSteerUpdate;

    Lwp3Gt4();
    ~Lwp3Gt4();

    /**
     * @brief Blocks and searches for the Porsche until found or interrupted.
     */
    bool connect();
    void disconnect();
    void autoCalibrate();

    void setSteer(int32_t relative_angle);
    void setDrive(int8_t speed); 
    void stop();

    int32_t getMinSteer() const { return minRelative; }
    int32_t getMaxSteer() const { return maxRelative; }
    bool isConnected();

private:
    SimpleBLE::Peripheral porsche;
    
    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<bool> telemetryActive{false};

    int32_t hardwareCenter = 0; 
    int32_t minRelative = -60; 
    int32_t maxRelative = 60;  
    
    const uint8_t PORT_STEER   = 0x34;
    const uint8_t PORT_DRIVE_L = 0x32;
    const uint8_t PORT_DRIVE_R = 0x33;

    const std::string SERVICE_UUID = "00001623-1212-efde-1623-785feabcd123";
    const std::string CHAR_UUID    = "00001624-1212-efde-1623-785feabcd123";

    void setupNotifications();
    void sendRaw(std::vector<uint8_t> data);
    void setSteerRaw(int32_t absolute_angle, uint8_t speed = 40);
    
    int32_t sweep_to_limit(int8_t speed, uint8_t power, const std::string& label);
    void waitForMovement(int32_t target_absolute, int32_t tolerance = 2, int timeout_ms = 3000);
};