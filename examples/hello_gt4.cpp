#include "Lwp3Gt4.hpp"
#include <iostream>
#include <thread>

int main() {
    Lwp3Gt4 car;
    std::cout << "--- LWP3-GT4-SDK Test Suite ---" << std::endl;

    try {
        // This will now block and wait for you to press the button on the car
        car.connect();

        car.onSteerUpdate = [](int32_t rel_pos) {
            std::cout << "\r[Tele] Relative: " << rel_pos << "°    " << std::flush;
        };

        car.autoCalibrate();

        std::cout << "\n[Main] Testing boundaries..." << std::endl;
        
        car.setSteer(car.getMinSteer());
        std::this_thread::sleep_for(std::chrono::seconds(2));

        car.setSteer(car.getMaxSteer());
        std::this_thread::sleep_for(std::chrono::seconds(2));

        car.setSteer(0);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        car.disconnect();
        std::cout << "[Main] Complete. Porsche is safe." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Fatal Error: " << e.what() << std::endl;
    }

    return 0;
}