# LWP3-GT4-SDK

A high-performance C++17 library for the **LEGO Technic Porsche GT4 e-Performance (42176)**.

***

### Disclaimer
*LEGO® is a trademark of the LEGO Group of companies. Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG. This project is not sponsored, authorized, or endorsed by the LEGO Group or Porsche AG. This SDK is an independent community project developed for interoperability purposes under nominative fair use.*

***

## Scope
This SDK provides a professional, thread-safe interface to control the Porsche GT4 Hub via the LEGO Wireless Protocol v3 (LWP3). It abstracts low-level BLE hex commands into a clean, normalized C++ API, making it suitable for both hobbyist projects and advanced autonomous driving research.

### Key Features
* **Zero-Center Calibration:** Automated mechanical homing routine that finds physical steering limits and maps them to a normalized 0-centered degree system.
* **Synchronized Drive (Virtual Port):** Automatically pairs rear motors into a single logical entity to ensure perfectly simultaneous acceleration and braking.
* **Differential Drive Support:** Allows individual motor speed control for torque vectoring and drifting maneuvers.
* **Real-time Telemetry:** Asynchronous callbacks for:
    * Steering Position (0-centered degrees)
    * Battery Health (0-100%)
    * Signal Strength (RSSI in dBm)
    * Hub Button State (Physical green button events)

## Dependencies
* **C++17 Compiler** (GCC 9+, Clang 10+, or MSVC 2019+)
* **SimpleBLE:** A cross-platform Bluetooth library.
    * *Ubuntu/Debian:* `sudo apt install libdbus-1-dev`
    * *Other Platforms:* Refer to the [SimpleBLE Documentation](https://github.com/OpenBluetoothToolbox/SimpleBLE).

## Integration as a Git Submodule
To integrate this SDK into your existing workspace:

1.  **Add the submodule:**
    ```bash
    git submodule add [https://github.com/yourusername/lwp3-gt4-sdk.git](https://github.com/yourusername/lwp3-gt4-sdk.git) third_party/lwp3-gt4-sdk
    git submodule update --init --recursive
    ```

2.  **Update your `CMakeLists.txt`:**
    ```cmake
    add_subdirectory(third_party/lwp3-gt4-sdk)
    target_link_libraries(your_project_name PRIVATE Lwp3Gt4SDK)
    ```

## Telemetry & Notifications
The SDK uses a callback-based system via `std::function`. You should register your listeners before calling `connect()` to ensure you capture the initial handshake data.

### Available Callbacks
* `onSteerUpdate`: Returns `int32_t` (degrees relative to center).
* `onBatteryUpdate`: Returns `uint8_t` (percentage 0-100).
* `onButtonUpdate`: Returns `bool` (true = pressed, false = released).
* `onRssiUpdate`: Returns `int8_t` (signal strength in dBm).

## Usage Example

```cpp
#include "Lwp3Gt4.hpp"
#include <iostream>
#include <thread>

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
            std::cout << "\rSteering Position: " << pos << "°    " << std::flush;
        };

        // 2. Connect to the Hub
        if (car.connect(mac)) {
            
            // 3. Perform automatic calibration
            car.autoCalibrate();

            // 4. Command Movement
            car.setSteer(20);  // Turn 20 degrees right
            car.setDrive(50);  // 50% power forward
            
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
```

## Documentation
The header file Lwp3Gt4.hpp is fully documented using Doxygen-style comments. You can generate a full HTML reference manual by running:

```bash
doxygen Doxyfile
```

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.