/**
 * @file hello_gt4.cpp
 * @brief Demonstration of the LWP3-GT4-SDK capabilities.
 */

#include "Lwp3Gt4.hpp"
#include <iostream>
#include <thread>

// Enable time literals for clean duration syntax
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    Lwp3Gt4 car;
    
    // Check if user provided a custom MAC address as CLI argument
    std::string mac = "28:3C:90:9C:82:14"; 
    if (argc > 1) mac = argv[1];

    std::cout << "--- LWP3-GT4-SDK Professional Test Suite ---" << std::endl;

    try {
        // Register telemetry callbacks
        car.onBatteryUpdate = [](uint8_t level) {
            std::cout << "\n[Hub] Battery Status: " << (int)level << "%" << std::endl;
        };

        car.onButtonUpdate = [](bool pressed) {
            std::cout << "\n[Hub] Physical Button: " << (pressed ? "DOWN" : "UP") << std::endl;
        };

        car.onRssiUpdate = [](int8_t rssi) {
            std::cout << "[Hub] Signal Strength: " << (int)rssi << " dBm" << std::endl;
        };

        car.onSteerUpdate = [](int32_t rel_pos) {
            std::cout << "\r[Tele] Wheel Angle: " << rel_pos << "°    " << std::flush;
        };

        // Initialize connection and mechanical alignment
        car.connect(mac);
        car.autoCalibrate();

        std::cout << "\n[Main] Sequence active. Press Car Button to test callback, or wait 10s..." << std::endl;
        
        // Monitoring loop
        for(int i = 0; i < 10; i++) {
            std::this_thread::sleep_for(1s);
            std::cout << "." << std::flush;
        }

        // Graceful exit
        car.disconnect();
        std::cout << "\n[Main] Test finished. Car disconnected safely." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\n[FATAL] Exception: " << e.what() << std::endl;
    }

    return 0;
}