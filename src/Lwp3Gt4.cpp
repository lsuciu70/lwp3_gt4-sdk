#include "Lwp3Gt4.hpp"
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

Lwp3Gt4::Lwp3Gt4() {}
Lwp3Gt4::~Lwp3Gt4() { disconnect(); }

bool Lwp3Gt4::connect() {
    while (true) {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) {
            std::cerr << "\r[SDK] No Bluetooth adapters found!" << std::flush;
            std::this_thread::sleep_for(1s);
            continue;
        }

        auto adapter = adapters[0];
        std::cout << "\r[SDK] Searching for Porsche GT4 (Press Hub Button)... " << std::flush;
        
        adapter.scan_for(1000); // Scan in 1s bursts
        auto results = adapter.scan_get_results();

        for (auto& p : results) {
            if (p.address() == "28:3C:90:9C:82:14") {
                porsche = p;
                std::cout << "\n[SDK] Hub detected! Connecting..." << std::endl;
                
                try {
                    porsche.connect();
                    std::this_thread::sleep_for(1500ms); 
                    setupNotifications();
                    return true;
                } catch (const std::exception& e) {
                    std::cerr << "\n[SDK] Connection failed: " << e.what() << ". Retrying..." << std::endl;
                }
            }
        }
        // Small breather before next scan burst
        std::this_thread::sleep_for(500ms);
    }
    return false;
}

void Lwp3Gt4::setupNotifications() {
    try {
        porsche.notify(SERVICE_UUID, CHAR_UUID, [this](SimpleBLE::ByteArray data) {
            auto* raw = (uint8_t*)data.data();
            if (data.size() >= 8 && raw[2] == 0x45 && raw[3] == PORT_STEER) {
                rawSteerPos = *(int32_t*)(&raw[4]);
                telemetryActive = true;
                if (onSteerUpdate) onSteerUpdate(rawSteerPos.load() - hardwareCenter);
            }
        });
        sendRaw({0x0A, 0x00, 0x41, PORT_STEER, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
    } catch (...) {}
}

void Lwp3Gt4::setSteerRaw(int32_t absolute_angle, uint8_t speed) {
    std::vector<uint8_t> cmd = {0x0E, 0x00, 0x81, PORT_STEER, 0x11, 0x0D};
    uint8_t* p = (uint8_t*)&absolute_angle;
    for(int i=0; i<4; i++) cmd.push_back(p[i]);
    cmd.insert(cmd.end(), {speed, 0x64, 0x7E, 0x03});
    sendRaw(cmd);
}

void Lwp3Gt4::waitForMovement(int32_t target_absolute, int32_t tolerance, int timeout_ms) {
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        if (std::abs(rawSteerPos.load() - target_absolute) <= tolerance) return;
        std::this_thread::sleep_for(100ms);
        elapsed += 100;
    }
}

int32_t Lwp3Gt4::sweep_to_limit(int8_t speed, uint8_t power, const std::string& label) {
    std::cout << "[SDK] Sweeping " << label << "... " << std::flush;
    sendRaw({0x09, 0x00, 0x81, PORT_STEER, 0x11, 0x07, (uint8_t)speed, power, 0x03});
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

    sendRaw({0x09, 0x00, 0x81, PORT_STEER, 0x11, 0x07, 0x00, 0x64, 0x03});
    std::this_thread::sleep_for(300ms);
    
    int32_t final_p = rawSteerPos.load();
    std::cout << "Stalled at " << final_p << "°" << std::endl;
    return final_p;
}

void Lwp3Gt4::autoCalibrate() {
    std::cout << "[SDK] Waiting for telemetry..." << std::endl;
    for(int i=0; i<30 && !telemetryActive; i++) std::this_thread::sleep_for(100ms);
    if (!telemetryActive) return;

    int32_t initialRawPos = rawSteerPos.load();
    hardwareCenter = initialRawPos; 

    int32_t rawLeft = sweep_to_limit(-30, 40, "LEFT");
    std::this_thread::sleep_for(500ms);

    std::cout << "[SDK] Resetting to initial position... " << std::flush;
    setSteerRaw(initialRawPos, 50);
    waitForMovement(initialRawPos, 3, 4000);
    std::cout << "OK" << std::endl;
    std::this_thread::sleep_for(500ms);

    int32_t rawRight = sweep_to_limit(30, 40, "RIGHT");

    int32_t newCenter = (rawLeft + rawRight) / 2;
    minRelative = std::min(rawLeft, rawRight) - newCenter;
    maxRelative = std::max(rawLeft, rawRight) - newCenter;
    hardwareCenter = newCenter;

    std::cout << "[SDK] Calibration Complete. True Center: " << hardwareCenter << std::endl;
    
    std::cout << "[SDK] Centering wheels... " << std::flush;
    setSteer(0);
    waitForMovement(hardwareCenter, 1, 3000);
    std::cout << "OK" << std::endl;
}

void Lwp3Gt4::setSteer(int32_t relative_angle) {
    int32_t clamped = std::clamp(relative_angle, minRelative, maxRelative);
    setSteerRaw(hardwareCenter + clamped, 50);
}

void Lwp3Gt4::setDrive(int8_t speed) {
    int8_t clamped = std::clamp(speed, (int8_t)-100, (int8_t)100);
    for (uint8_t port : {PORT_DRIVE_L, PORT_DRIVE_R}) {
        sendRaw({0x09, 0x00, 0x81, port, 0x11, 0x07, (uint8_t)clamped, 0x64, 0x03});
    }
}

void Lwp3Gt4::stop() { setDrive(0); }

void Lwp3Gt4::sendRaw(std::vector<uint8_t> data) {
    if (!porsche.is_connected()) return;
    SimpleBLE::ByteArray bytes(data.begin(), data.end());
    try { porsche.write_request(SERVICE_UUID, CHAR_UUID, bytes); } catch (...) {}
}

void Lwp3Gt4::disconnect() {
    if (porsche.is_connected()) {
        stop();
        std::this_thread::sleep_for(500ms);
        porsche.disconnect();
    }
}

bool Lwp3Gt4::isConnected() { return porsche.is_connected(); }