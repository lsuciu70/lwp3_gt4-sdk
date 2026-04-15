#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "../include/Lwp3Gt4.hpp"

int main(int argc, char** argv) {
    std::cout << "--- LWP3 GT4 SDK v3.0 Latency Profiler ---\n";

    std::string mac = "28:3C:90:9C:82:14"; 
    if (argc > 1) {
        mac = argv[1];
    } else {
        std::cout << "Usage: ./ble_latency_test <MAC_ADDRESS>\n";
        std::cout << "Using default mock MAC: " << mac << "\n\n";
    }

    LWP3::PorscheGt4 car;

    std::cout << "Connecting...\n";
    if (!car.connect(mac)) {
        std::cerr << "Connection failed.\n";
        return 1;
    }

    std::cout << "Calibrating chassis...\n";
    car.autoCalibrate();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    const int NUM_SAMPLES = 20;
    std::vector<double> latencies_ms;
    latencies_ms.reserve(NUM_SAMPLES);

    std::cout << "\nStarting Ping-Pong Latency Test (" << NUM_SAMPLES << " iterations)...\n";
    std::cout << "Targeting +/- 50 degree sweeps.\n\n";

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        int8_t target_steer = (i % 2 == 0) ? 50 : -50;

        // 1. Mark the start time and fire the command
        auto t_start = std::chrono::steady_clock::now();
        car.setCommand(0, target_steer);

        bool target_reached = false;

        // 2. High-frequency poll until the hardware catches up
        while (!target_reached) {
            LWP3::Telemetry t;

            // Drain the queue to get the absolute latest state
            while (car.pollTelemetry(t)) {
                // If we are within 5 degrees of the target, consider it "reached"
                if (std::abs(t.steer_pos - target_steer) <= 5) {
                    target_reached = true;
                    break;
                }
            }

            // Yield slightly to prevent 100% CPU lockup while waiting for BLE
            if (!target_reached) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // 3. Mark the end time
        auto t_end = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        latencies_ms.push_back(elapsed_ms);
        std::cout << "Iteration " << (i + 1) << "/" << NUM_SAMPLES
                  << " | Target: " << (int)target_steer << " | Latency: " << elapsed_ms << " ms\n";

        // Let the motor settle before the next violent sweep
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // ---------------------------------------------------------
    // STATS CALCULATION
    // ---------------------------------------------------------
    if (!latencies_ms.empty()) {
        double sum = std::accumulate(latencies_ms.begin(), latencies_ms.end(), 0.0);
        double mean = sum / latencies_ms.size();

        auto minmax = std::minmax_element(latencies_ms.begin(), latencies_ms.end());
        double min_val = *minmax.first;
        double max_val = *minmax.second;

        // Calculate a rough P99 (or P90 given the small sample size)
        std::sort(latencies_ms.begin(), latencies_ms.end());
        size_t p90_idx = static_cast<size_t>(0.90 * latencies_ms.size());
        double p90_val = latencies_ms[std::min(p90_idx, latencies_ms.size() - 1)];

        std::cout << "\n=== LATENCY PROFILE REPORT ===\n";
        std::cout << "Samples: " << NUM_SAMPLES << "\n";
        std::cout << "Mean:    " << mean << " ms\n";
        std::cout << "Min:     " << min_val << " ms\n";
        std::cout << "Max:     " << max_val << " ms\n";
        std::cout << "P90:     " << p90_val << " ms\n";
        std::cout << "==============================\n\n";
    }

    std::cout << "Test complete. Shutting down...\n";
    car.setCommand(0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    car.disconnect();

    return 0;
}
