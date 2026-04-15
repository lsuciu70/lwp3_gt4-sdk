/**
 * @file hello_gt4.cpp
 * @brief Autonomous Racing Test Application for LWP3-GT4-SDK.
 */

#include "Lwp3Gt4.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <iomanip>
#include <cmath>

using namespace std::chrono_literals;

// --- Global State for Terminal Dashboard ---
std::atomic<int32_t> currentSteer{0};
std::atomic<uint8_t> currentBattery{0};

std::atomic<double> latencyEma{0.0};
const double EMA_ALPHA = 0.1; 

void printTelemetry(LWP3::PorscheGt4& car, const std::string& phase) {
    if (!car.isLinkHealthy()) {
        std::cerr << "\n\n[WARN] BLE Link degraded! (Did the Hub brownout from low battery?)" << std::endl;
        exit(-1);
    }
    std::cout << "\r[" << std::left << std::setw(12) << phase << "] "
              << "Bat: " << std::setw(3) << (int)currentBattery.load() << "% | "
              << "Steer: " << std::setw(3) << currentSteer.load() << "° | "
              << "Latency: " << std::fixed << std::setprecision(1) << latencyEma.load() << "ms    " 
              << std::flush;
}

int main(int argc, char* argv[]) {
    LWP3::PorscheGt4 car;
    std::string mac = "28:3C:90:9C:82:14"; 
    if (argc > 1) mac = argv[1];

    std::cout << "=== LWP3-GT4-SDK Autonomous Slalom Test ===\n";

    try {
        car.onBatteryUpdate = [](uint8_t level) { currentBattery = level; };
        car.onSteerUpdate = [](int32_t pos) { currentSteer = pos; };
        car.onLatencyProfileUpdate = [](uint64_t raw_ms) {
            double current_ema = latencyEma.load();
            if (current_ema == 0.0) latencyEma.store((double)raw_ms); 
            else latencyEma.store((raw_ms * EMA_ALPHA) + (current_ema * (1.0 - EMA_ALPHA)));
        };

        if (!car.connect(mac)) {
            std::cerr << "[FATAL] Could not connect." << std::endl;
            return -1;
        }

        std::cout << "[Main] Connected! Starting hardware calibration...\n";
        if (!car.autoCalibrate()) {
            std::cerr << "[FATAL] Calibration failed." << std::endl;
            car.disconnect();
            return -1;
        }
        
        // --- Pre-Flight Power Check ---
        std::this_thread::sleep_for(500ms); // Give telemetry a moment to sync
        int batLevel = currentBattery.load();
        std::cout << "[Main] Calibration OK. Pre-Flight Battery: " << batLevel << "%\n";
        
        if (batLevel < 40) {
            std::cerr << "\n>>>>>>>>>>>>>>>>>> [WARNING] <<<<<<<<<<<<<<<<<<<<\n"
                      << "Battery is critically low for track racing!\n"
                      << "High-torque acceleration may cause a voltage sag,\n"
                      << "resulting in a BLE brownout and sudden disconnect.\n"
                      << ">>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<\n\n";
            std::this_thread::sleep_for(2s); // Force user to read it
        }

        std::cout << "[Main] Ready for Track Test.\n\n";
        std::this_thread::sleep_for(1s);

        // --- Phase 1: Smooth Acceleration ---
        for (int speed = 0; speed <= 50; speed += 2) {
            car.updateControl(speed, 0);
            printTelemetry(car, "Accelerating");
            std::this_thread::sleep_for(50ms);
        }

        // --- Phase 2: Smooth Slalom (Sine Wave Interpolation) ---
        int max_steer = 45;
        for (double t = 0; t <= M_PI * 3; t += 0.15) {
            int steer_angle = static_cast<int>(std::sin(t) * max_steer);
            car.updateControl(50, steer_angle);
            printTelemetry(car, "Slalom");
            std::this_thread::sleep_for(50ms);
        }

        // --- Phase 3: Smooth Braking ---
        for (int speed = 50; speed >= 0; speed -= 2) {
            car.updateControl(speed, 0);
            printTelemetry(car, "Braking");
            std::this_thread::sleep_for(50ms);
        }

        std::cout << "\n\n[Main] Track run complete. Shutting down.\n";
        car.stop();
        std::this_thread::sleep_for(500ms);
        car.disconnect();

    } catch (const std::exception& e) {
        std::cerr << "\n[FATAL Exception] " << e.what() << std::endl;
    }

    return 0;
}