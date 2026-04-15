#include <atomic>
#include <cmath>
#include <iostream>
#include <thread>

#include "../include/Lwp3Gt4.hpp"

std::atomic<bool> running{true};

void autonomyLoop(LWP3::PorscheGt4& car) {
    std::cout << "[AUTO] Autonomy Control Thread Started (50Hz)\n";
    auto period = std::chrono::milliseconds(20);
    int tick = 0;

    while (running) {
        auto start = std::chrono::steady_clock::now();

        LWP3::Telemetry t;
        int latest_steer = 0;
        uint8_t battery = 0;
        bool link_ok = true;
        bool got_data = false;

        while (car.pollTelemetry(t)) {
            latest_steer = t.steer_pos;
            battery = t.battery_level;
            link_ok = t.link_healthy;
            got_data = true;
        }

        // --- NEW: Safety Abort ---
        if (got_data && !link_ok) {
            std::cerr << "[AUTO] CRITICAL: BLE Link lost! Aborting sequence.\n";
            running = false;
            break;
        }

        float time_sec = tick * 0.02f;
        int32_t target_steer = static_cast<int32_t>(80.0f * std::sin(time_sec * 2.0f));

        car.setCommand(0, target_steer);

        if (tick % 50 == 0 && got_data) {
            std::cout << "[AUTO] Target: " << target_steer << " | Read: " << latest_steer
                      << " | Battery: " << (int)battery << "% \n";
        }

        tick++;
        std::this_thread::sleep_until(start + period);
    }
}

void mockBleHardwareLoop(LWP3::PorscheGt4& car) {
    std::cout << "[HARDWARE] BLE RX Thread Started (~66Hz)\n";
    auto period = std::chrono::milliseconds(15);
    int fake_sensor_val = -100;
    bool going_up = true;

    while (running) {
        auto start = std::chrono::steady_clock::now();

        if (going_up)
            fake_sensor_val += 2;
        else
            fake_sensor_val -= 2;
        if (fake_sensor_val >= 100) going_up = false;
        if (fake_sensor_val <= -100) going_up = true;

        car.mockBleNotification(fake_sensor_val);

        std::this_thread::sleep_until(start + period);
    }
}

int main() {
    std::cout << "--- LWP3 GT4 SDK Async Pipeline Test ---\n";

    LWP3::PorscheGt4 car;

    std::string mac = "28:3C:90:9C:82:14"; 
    car.connect(mac);
    car.autoCalibrate();

    std::thread auto_thread(autonomyLoop, std::ref(car));
    std::thread hw_thread(mockBleHardwareLoop, std::ref(car));

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout << "\n[MAIN] Stopping autonomy loop...\n";
    running = false;

    auto_thread.join();
    hw_thread.join();

    std::cout << "[MAIN] Returning steering to center...\n";
    car.setCommand(0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    car.disconnect();

    std::cout << "--- Pipeline Test Complete ---\n";
    return 0;
}
