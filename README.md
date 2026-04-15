# LWP3-GT4-SDK v2.0
## Technic Porsche GT4 e-Performance ADAS SDK

A high-performance C++20 SDK designed for autonomous racing and Advanced Driver Assistance Systems (ADAS) using the LEGO Wireless Protocol v3 (LWP3). 

This SDK provides a robust, thread-safe interface to the LEGO Technic Porsche GT4, featuring an asynchronous architecture that decouples high-level logic from low-level Bluetooth communication.

***

### Disclaimer
*LEGO® is a trademark of the LEGO Group of companies. Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG. This project is not sponsored, authorized, or endorsed by the LEGO Group or Porsche AG. This SDK is an independent community project developed for interoperability purposes under nominative fair use.*

***

### 🏎️ New in v2.0: The ADAS Architecture
The SDK has been re-engineered from the ground up to support real-time robotics requirements:

* **Triple-Threaded Execution:**
    1.  **Control Loop (50Hz):** A dedicated high-priority thread that processes steering/throttle logic and safety watchdogs every 20ms.
    2.  **Hardware TX Pipe (2ms):** A fast-polling mailbox consumer that pushes the freshest commands to the Bluetooth stack without blocking the logic thread.
    3.  **Telemetry Dispatcher:** An isolated thread for user-defined callbacks (battery, steering, latency), ensuring sensor data processing never induces jitter in the driving loops.
* **Mailbox Pattern:** Prevents "packet flooding" by ensuring only the latest control state is sent to the car, reducing Bluetooth congestion and latency.
* **Mechanical-Aware Calibration:** An intelligent 5-step sweep algorithm that detects physical chassis limits while accounting for motor-clutch slippage and static friction "wedging."
* **ADAS Link Watchdog:** Real-time monitoring of transmission success. The car executes a failsafe emergency stop if the Bluetooth link degrades for more than 200ms.

---

### 📦 Key Components
* **`PorscheGt4::connect()`**: Robust scanning and connection handler.
* **`PorscheGt4::autoCalibrate()`**: Automatic center-finding and limit detection.
* **`PorscheGt4::updateControl()`**: Thread-safe command interface for throttle (-100 to 100) and steering (relative degrees).
* **`Virtual Port Optimization`**: Automatically links drive motors for optimized packet delivery during linear movement.

---

### 🛠️ Build & Installation
**Requirements:**
* C++20 compliant compiler (GCC 13+ recommended)
* [SimpleBLE](https://github.com/OpenBluetoothToolbox/SimpleBLE) library installed

**Compilation:**
```bash
mkdir build && cd build
cmake ..
make
```

**Running the Test Suite:**
```bash
./hello_gt4 [MAC_ADDRESS]
```

---

### 📖 API Example
```cpp
#include "Lwp3Gt4.hpp"

int main() {
    LWP3::PorscheGt4 car;
    
    // 1. Telemetry Hooks
    car.onBatteryUpdate = [](uint8_t level) { 
        std::cout << "Battery: " << (int)level << "%" << std::endl; 
    };

    // 2. Connect & Calibrate
    if (!car.connect("28:3C:90:9C:82:14")) return -1;
    if (!car.autoCalibrate()) return -1;

    // 3. Drive Phase
    // updateControl(throttle, relative_steer)
    car.updateControl(50, -15); // 50% power, 15 degrees left
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 4. Safe Shutdown
    car.stop();
    car.disconnect();
    return 0;
}
```

---

### ⚠️ Critical Hardware Safety
* **Voltage Brownouts:** The Technic Hub requires significant current for the GT4's dual XL motors. If the battery level falls below **40%**, high-torque acceleration may cause a voltage sag that resets the Bluetooth chip, resulting in a sudden disconnect.
* **Charging Isolation:** The Hub hardware disables motor PWM outputs while the USB-C cable is connected. The SDK will remain connected and show telemetry, but the car will not move while charging.
* **Internal Clutch:** The steering motor contains a white safety clutch. The SDK calibration is designed to detect the "click" of this clutch to prevent gear damage while finding chassis limits.

---

### 📂 Project Structure
* `include/Lwp3Constants.hpp`: LWP3 protocol definitions and UUIDs.
* `include/Lwp3Gt4.hpp`: SDK Interface and Class definition.
* `src/Lwp3Gt4.cpp`: Core logic, threading, and hardware implementation.
* `examples/hello_gt4.cpp`: Reference ADAS slalom test application.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

---
*Author: lsuciu70* *Version: 2.0.0 "The ADAS Refactor"*
