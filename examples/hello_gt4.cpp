/**
 * @file hello_gt4.cpp
 * @brief Demonstration of the LWP3-GT4-SDK capabilities.
 */

#include <iostream>
#include <thread>

#include "Lwp3Gt4.hpp"

// Enable time literals for clean duration syntax
using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    LWP3::PorscheGt4 car;

    // Provide your car's MAC address via CLI or use a local variable
    std::string mac = "28:3C:90:9C:82:14";
    if (argc > 1) mac = argv[1];

    try {
        // 1. Register Telemetry Callbacks (Simple Lambdas)
        car.onBatteryUpdate = [](uint8_t level) {
            std::cout << "Battery: " << (int)level << "%" << std::endl;
        };

        car.onSteerUpdate = [](int32_t pos) {
            std::cout << "\rSteering Position: " << pos << "°" << std::endl;
        };

        // 2. Connect to the Hub
        if (car.connect(mac)) {
            // 3. Perform automatic calibration and print margins
            car.autoCalibrate();
            std::cout << "Min Steer: " << car.getMinSteer() << "°" << std::endl;
            std::cout << "Max Steer: " << car.getMaxSteer() << "°" << std::endl;

            // 4. Command Movement
            car.setSteer(40);  // Turn 40 degrees right
            car.setDrive(50);  // 50% power forward
            car.setSteer(0);   // Head straight

            std::this_thread::sleep_for(2s);

            // 5. Cleanup
            car.stop();
            car.disconnect();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
