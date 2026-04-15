#include <cmath>
#include <iostream>
#include <thread>

#include "Lwp3Gt4.hpp"  // No more ../include/!

using namespace std::chrono_literals;

void holdCommand(LWP3::PorscheGt4& car, int8_t speed, int32_t steer, int ms) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        car.sendCommand({steer, speed});
        std::this_thread::sleep_for(20ms);
    }
}

int main(int argc, char** argv) {
    std::string mac = (argc > 1) ? argv[1] : "28:3C:90:9C:82:14";
    LWP3::PorscheGt4 car;

    if (!car.connect(mac)) return 1;
    car.autoCalibrate();

    std::cout << "Accelerating...\n";
    for (int s = 0; s <= 40; s += 2) holdCommand(car, s, 0, 50);

    std::cout << "Starting Smooth Slalom...\n";
    for (int i = 0; i < 300; ++i) {
        float t = i * 0.02f;

        // Dynamic envelope to prevent the "last turn brutal snap"
        float envelope = 1.0f;
        if (i > 250) envelope = 1.0f - ((i - 250) / 50.0f);

        int32_t angle = static_cast<int32_t>(envelope * 50.0f * std::sin(t * 3.5f));
        car.sendCommand({angle, 40});
        std::this_thread::sleep_for(20ms);
    }

    std::cout << "Smooth Stop...\n";
    for (int s = 40; s >= 0; s -= 2) holdCommand(car, s, 0, 50);

    holdCommand(car, 0, 0, 500);
    car.disconnect();
    return 0;
}
