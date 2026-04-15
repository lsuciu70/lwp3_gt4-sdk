/**
 * @file ble_latency_test.cpp
 * @brief v4.0 Deterministic Latency Profiler.
 */

#include <iomanip>
#include <iostream>
#include <thread>

#include "../include/Lwp3Gt4.hpp"

int main(int argc, char** argv) {
    std::string mac = (argc > 1) ? argv[1] : "28:3C:90:9C:82:14";
    LWP3::PorscheGt4 car;

    if (!car.connect(mac)) return 1;
    car.autoCalibrate();

    std::cout << "Profiling physical latency (20 cycles)...\n";

    for (int i = 0; i < 20; ++i) {
        int target = (i % 2 == 0) ? 40 : -40;

        // Dispatch command
        car.sendCommand({target, 0});

        // Wait for physical wheels to arrive
        while (std::abs(car.getLatestTelemetry().steer_pos - target) > 3) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    auto stats = car.getLatencyStats();
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n--- FINAL HAL LATENCY PROFILE ---\n";
    std::cout << "Mean: " << stats.mean_ms << " ms\n";
    std::cout << "P50:  " << stats.p50_ms << " ms\n";
    std::cout << "P99:  " << stats.p99_ms << " ms\n";
    std::cout << "---------------------------------\n";

    car.disconnect();
    return 0;
}
