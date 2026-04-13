#include "Lwp3Gt4.hpp"
#include <thread>
#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {}
PorscheGt4::~PorscheGt4() { disconnect(); }

bool PorscheGt4::connect(const std::string& address) {
    while (true) {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) { std::this_thread::sleep_for(1s); continue; }

        auto adapter = adapters[0];
        std::cout << "\r[SDK] Searching for Porsche at " << address << "... " << std::flush;
        
        adapter.scan_for(1000);
        auto results = adapter.scan_get_results();

        for (auto& p : results) {
            if (p.address() == address) {
                porsche = p;
                std::cout << "\n[SDK] Hub detected! Connecting..." << std::endl;
                try {
                    porsche.connect();
                    std::this_thread::sleep_for(1500ms); 
                    setupNotifications();
                    
                    sendRaw({LEN_VIRTUAL_PORT_SETUP, 0x00, VIRTUAL_PORT_SETUP, 0x01, PORT_DRIVE_L, PORT_DRIVE_R});
                    sendRaw({LEN_HUB_PROP_SETUP, 0x00, HUB_PROPERTIES, BATTERY_VOLTAGE, ENABLE_UPDATES});
                    sendRaw({LEN_HUB_PROP_SETUP, 0x00, HUB_PROPERTIES, BUTTON_STATE, ENABLE_UPDATES});
                    sendRaw({LEN_HUB_PROP_SETUP, 0x00, HUB_PROPERTIES, RSSI, ENABLE_UPDATES});
                    
                    return true;
                } catch (...) {
                    std::cerr << "\n[SDK] Connection failed. Retrying..." << std::endl;
                }
            }
        }
        std::this_thread::sleep_for(500ms);
    }
}

void PorscheGt4::setupNotifications() {
    try {
        porsche.notify(SERVICE_UUID, CHAR_UUID, [this](SimpleBLE::ByteArray data) {
            auto* raw = (uint8_t*)data.data();
            if (data.size() < 3) return;

            switch (raw[2]) {
                case PORT_VALUE_SINGLE:
                    if (raw[3] == PORT_STEER) {
                        rawSteerPos.store(*(int32_t*)(&raw[4]));
                        telemetryActive.store(true);
                        if (onSteerUpdate) onSteerUpdate(rawSteerPos.load() - hardwareCenter);
                    }
                    break;

                case HUB_PROPERTIES:
                    if (raw[4] != UPDATE) break;
                    if (raw[3] == BATTERY_VOLTAGE) {
                        batteryLevel.store(raw[5]); 
                        if (onBatteryUpdate) onBatteryUpdate(batteryLevel.load());
                    } else if (raw[3] == BUTTON_STATE) {
                        buttonPressed.store(raw[5] == 0x01);
                        if (onButtonUpdate) onButtonUpdate(buttonPressed.load());
                    } else if (raw[3] == RSSI) {
                        rssiValue.store((int8_t)raw[5]);
                        if (onRssiUpdate) onRssiUpdate(rssiValue.load());
                    }
                    break;

                case HUB_ATTACHED_IO:
                    if (raw[4] == 0x02) { 
                        if (raw[7] == PORT_DRIVE_L && raw[8] == PORT_DRIVE_R) {
                            virtualDrivePort.store(raw[3]);
                        }
                    }
                    break;
            }
        });
        sendRaw({LEN_PORT_INPUT_FORMAT, 0x00, PORT_INPUT_FORMAT_SETUP, PORT_STEER, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
    } catch (...) {}
}

void PorscheGt4::setDrive(int8_t speed) {
    uint8_t v_port = virtualDrivePort.load();
    if (v_port != 0xFF) {
        int8_t clamped = std::clamp(speed, (int8_t)-100, (int8_t)100);
        sendRaw({LEN_MOTOR_POWER, 0x00, PORT_OUTPUT_COMMAND, v_port, 0x11, START_POWER, (uint8_t)clamped, 0x64, 0x03});
    } else {
        setDrive(speed, speed);
    }
}

void PorscheGt4::setDrive(int8_t leftSpeed, int8_t rightSpeed) {
    int8_t cL = std::clamp(leftSpeed, (int8_t)-100, (int8_t)100);
    int8_t cR = std::clamp(rightSpeed, (int8_t)-100, (int8_t)100);
    sendRaw({LEN_MOTOR_POWER, 0x00, PORT_OUTPUT_COMMAND, PORT_DRIVE_L, 0x11, START_POWER, (uint8_t)cL, 0x64, 0x03});
    sendRaw({LEN_MOTOR_POWER, 0x00, PORT_OUTPUT_COMMAND, PORT_DRIVE_R, 0x11, START_POWER, (uint8_t)cR, 0x64, 0x03});
}

void PorscheGt4::setSteerRaw(int32_t absolute_angle, uint8_t speed) {
    std::vector<uint8_t> cmd = {LEN_GOTO_ABS_POS, 0x00, PORT_OUTPUT_COMMAND, PORT_STEER, 0x11, GOTO_ABS_POS};
    uint8_t* p = (uint8_t*)&absolute_angle;
    for(int i=0; i<4; i++) cmd.push_back(p[i]);
    cmd.insert(cmd.end(), {speed, 0x64, 0x7E, 0x03});
    sendRaw(SimpleBLE::ByteArray((const char*)cmd.data(), cmd.size()));
}

void PorscheGt4::autoCalibrate() {
    for(int i=0; i<30 && !telemetryActive.load(); i++) std::this_thread::sleep_for(100ms);
    if (!telemetryActive.load()) return;
    int32_t initialRawPos = rawSteerPos.load();
    hardwareCenter = initialRawPos; 
    int32_t rawLeft = sweep_to_limit(-30, 40, "LEFT");
    std::this_thread::sleep_for(500ms);
    setSteerRaw(initialRawPos, 50);
    waitForMovement(initialRawPos, 3, 4000);
    std::this_thread::sleep_for(500ms);
    int32_t rawRight = sweep_to_limit(30, 40, "RIGHT");
    int32_t newCenter = (rawLeft + rawRight) / 2;
    minRelative.store(std::min(rawLeft, rawRight) - newCenter);
    maxRelative.store(std::max(rawLeft, rawRight) - newCenter);
    hardwareCenter = newCenter;
    setSteer(0);
    waitForMovement(hardwareCenter, 1, 3000);
}

void PorscheGt4::waitForMovement(int32_t target_absolute, int32_t tolerance, int timeout_ms) {
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        if (std::abs(rawSteerPos.load() - target_absolute) <= tolerance) return;
        std::this_thread::sleep_for(100ms);
        elapsed += 100;
    }
}

int32_t PorscheGt4::sweep_to_limit(int8_t speed, uint8_t power, const std::string& label) {
    sendRaw({LEN_MOTOR_POWER, 0x00, PORT_OUTPUT_COMMAND, PORT_STEER, 0x11, START_POWER, (uint8_t)speed, power, 0x03});
    std::this_thread::sleep_for(600ms);
    int32_t last_pos = rawSteerPos.load();
    int stuck_count = 0;
    for (int i = 0; i < 80; i++) {
        std::this_thread::sleep_for(100ms);
        int32_t p = rawSteerPos.load();
        if (std::abs(p - last_pos) < 1) stuck_count++;
        else stuck_count = 0;
        if (stuck_count >= 4) break; 
        last_pos = p;
    }
    sendRaw({LEN_MOTOR_POWER, 0x00, PORT_OUTPUT_COMMAND, PORT_STEER, 0x11, START_POWER, 0x00, 0x64, 0x03});
    std::this_thread::sleep_for(300ms);
    return rawSteerPos.load();
}

void PorscheGt4::setSteer(int32_t relative_angle) {
    int32_t clamped = std::clamp(relative_angle, minRelative.load(), maxRelative.load());
    setSteerRaw(hardwareCenter + clamped, 50);
}

void PorscheGt4::stop() { 
    setDrive(0, 0); 
    sendRaw({LEN_MOTOR_POWER, 0x00, PORT_OUTPUT_COMMAND, PORT_STEER, 0x11, START_POWER, 0x00, 0x64, 0x03});
}

void PorscheGt4::sendRaw(SimpleBLE::ByteArray data) {
    if (!porsche.is_connected()) return;
    try { porsche.write_request(SERVICE_UUID, CHAR_UUID, data); } catch (...) {}
}

void PorscheGt4::disconnect() {
    if (porsche.is_connected()) {
        stop();
        std::this_thread::sleep_for(500ms);
        porsche.disconnect();
    }
}

bool PorscheGt4::isConnected() { return porsche.is_connected(); }
uint8_t PorscheGt4::getBatteryLevel() const { return batteryLevel.load(); }
bool PorscheGt4::isButtonPressed() const { return buttonPressed.load(); }
int32_t PorscheGt4::getMinSteer() const { return minRelative.load(); }
int32_t PorscheGt4::getMaxSteer() const { return maxRelative.load(); }

} // namespace LWP3