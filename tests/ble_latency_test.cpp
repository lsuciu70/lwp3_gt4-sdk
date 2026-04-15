#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

#include "Lwp3Gt4.hpp"

using namespace std::chrono;

static std::atomic<bool> running{true};

uint64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return uint64_t(ts.tv_sec) * 1'000'000'000ull + ts.tv_nsec;
}

struct Sample {
    uint64_t tx_ns;
    uint64_t rx_ns;
    int tx_val;
    int rx_val;
};

void signal_handler(int) {
    running = false;
    std::cout << "\n[CTRL+C] stopping...\n";
}

int main() {
    signal(SIGINT, signal_handler);

    LWP3::PorscheGt4 car;

    std::vector<Sample> samples;
    samples.reserve(10000);

    std::atomic<int> last_tx{0};
    std::atomic<uint64_t> last_tx_time{0};
    std::atomic<bool> latency_recorded{true};

    // --- RX callback ---
    car.onSteerUpdate = [&](int32_t pos) {
        uint64_t t = now_ns();

        if (!latency_recorded.load(std::memory_order_relaxed)) {
            int tx = last_tx.load(std::memory_order_relaxed);

            // Success condition: wheels physically reached within 4 degrees of target
            if (std::abs(pos - tx) <= 4) {
                uint64_t tx_t = last_tx_time.load(std::memory_order_relaxed);
                latency_recorded.store(true, std::memory_order_relaxed);

                Sample s;
                s.tx_ns = tx_t;
                s.rx_ns = t;
                s.tx_val = tx;
                s.rx_val = pos;
                samples.push_back(s);

                std::cout << "[SUCCESS] Target " << tx << " reached in " << (t - tx_t) / 1e6
                          << " ms" << std::endl;
            }
        }
        std::cout << "[RX] pos=" << pos << std::endl;
    };

    std::string mac = "28:3C:90:9C:82:14";

    std::cout << "Connecting to GT4..." << std::endl;
    if (!car.connect(mac)) {
        std::cerr << "Failed to connect.\n";
        return 1;
    }
    std::cout << "Connected.\nCalibrating steering...\n";

    if (!car.autoCalibrate()) {
        std::cerr << "Calibration failed.\n";
        car.disconnect();
        return 1;
    }
    std::cout << "Calibration done.\n";

    auto send_and_wait = [&](int steer, int duration_ms) {
        last_tx.store(steer, std::memory_order_relaxed);
        last_tx_time.store(now_ns(), std::memory_order_relaxed);
        latency_recorded.store(false, std::memory_order_relaxed);  // Arm the latency trigger

        car.updateControl(0, steer);
        std::cout << "[TX] steer=" << steer << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    };

    // Run Latency Test
    for (int i = 0; i < 10 && running; ++i) {
        send_and_wait(-40, 800);
        send_and_wait(40, 800);
    }

    std::cout << "Stopping...\n";
    car.stop();
    std::this_thread::sleep_for(200ms);

    car.disconnect();
    std::this_thread::sleep_for(500ms);

    // ============================================================
    // ANALYSIS
    // ============================================================

    std::vector<double> lat_ms;
    lat_ms.reserve(samples.size());

    for (auto& s : samples) {
        if (s.tx_ns == 0) continue;
        double ms = (s.rx_ns - s.tx_ns) / 1e6;
        if (ms > 0 && ms < 2000) lat_ms.push_back(ms);
    }

    if (lat_ms.empty()) {
        std::cout << "\nNo valid samples completed the turn target.\n";
        return 0;
    }

    std::sort(lat_ms.begin(), lat_ms.end());

    double mean = std::accumulate(lat_ms.begin(), lat_ms.end(), 0.0) / lat_ms.size();
    double p50 = lat_ms[lat_ms.size() * 0.50];
    double p95 = lat_ms[lat_ms.size() * 0.95];
    double p99 = lat_ms[lat_ms.size() * 0.99];

    std::cout << "\n=== PHYSICAL RESPONSE LATENCY ===\n";
    std::cout << "Completed turns: " << lat_ms.size() << "\n";
    std::cout << "Mean (time to reach target): " << mean << " ms\n";
    std::cout << "Median (p50): " << p50 << " ms\n";
    std::cout << "Max (p99): " << p99 << " ms\n";

    return 0;
}
