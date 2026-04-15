#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "../include/Lwp3Gt4.hpp"

// Helper to keep the watchdog alive by sending the command at 50Hz for a set duration
void holdCommand(LWP3::PorscheGt4& car, int8_t speed, int32_t steer, int duration_ms,
                 const std::string& phase) {
    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::milliseconds(duration_ms);
    auto period = std::chrono::milliseconds(20);  // 50Hz heartbeat

    while (std::chrono::steady_clock::now() < end) {
        car.setCommand(speed, steer);

        LWP3::Telemetry t;
        int latest_steer = steer;
        uint8_t batt = 0;
        bool has_data = false;

        // Always drain the queue completely so telemetry doesn't lag!
        while (car.pollTelemetry(t)) {
            latest_steer = t.steer_pos;
            batt = t.battery_level;
            has_data = true;
        }

        static int tick = 0;
        if (has_data && (tick++ % 15 == 0)) {
            std::cout << "[" << phase << "] Tgt: " << std::setw(3) << steer
                      << " | Read: " << std::setw(3) << latest_steer << " | Batt: " << (int)batt
                      << "%\n";
        }

        std::this_thread::sleep_for(period);
    }
}

int main(int argc, char** argv) {
    std::cout << "--- LWP3 GT4 SDK v3.0 Smooth Drive Demo ---\n";

    std::string mac = "28:3C:90:9C:82:14"; 
    if (argc > 1) {
        mac = argv[1];
    } else {
        std::cout << "Usage: ./hello_gt4 <MAC_ADDRESS>\n";
        std::cout << "Using default mock MAC: " << mac << "\n\n";
    }

    LWP3::PorscheGt4 car;

    if (!car.connect(mac)) {
        std::cerr << "Failed to connect to the vehicle.\n";
        return 1;
    }

    std::cout << "Running physical calibration sweep...\n";
    car.autoCalibrate();

    // ---------------------------------------------------------
    // 1. SMOOTH ACCELERATE SEQUENCE
    // ---------------------------------------------------------
    std::cout << "\n--- Accelerating ---\n";
    // Ramp up by 2% increments instead of 10% blocks
    for (int speed = 0; speed <= 50; speed += 2) {
        holdCommand(car, speed, 0, 40, "Accelerating");
    }

    // ---------------------------------------------------------
    // 2. SMOOTH SINE-WAVE SLALOM
    // ---------------------------------------------------------
    std::cout << "\n--- Sine-Wave Slalom ---\n";
    // 150 ticks * 20ms = 3 seconds of perfectly smooth slalom
    for (int tick = 0; tick < 150; ++tick) {
        float time_sec = tick * 0.02f;
        // Oscillate smoothly between -50 and 50 degrees
        int32_t steer_angle = static_cast<int32_t>(50.0f * std::sin(time_sec * 3.0f));

        // 20ms duration matches our 50Hz control loop perfectly
        holdCommand(car, 50, steer_angle, 20, "Slalom");
    }

    // Smooth return to center
    holdCommand(car, 50, 0, 500, "Centering");

    // ---------------------------------------------------------
    // 3. SMOOTH DECELERATE SEQUENCE
    // ---------------------------------------------------------
    std::cout << "\n--- Decelerating ---\n";
    for (int speed = 50; speed >= 0; speed -= 2) {
        holdCommand(car, speed, 0, 40, "Decelerating");
    }

    // ---------------------------------------------------------
    // 4. STOP & SHUTDOWN
    // ---------------------------------------------------------
    std::cout << "\n--- Stopping ---\n";
    holdCommand(car, 0, 0, 1000, "Stopping");

    car.disconnect();
    std::cout << "Disconnected successfully. Goodbye!\n";

    return 0;
}
