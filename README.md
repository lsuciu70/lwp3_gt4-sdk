# LWP3-GT4-SDK (v4.9)
### High-Performance robotic HAL for LEGO Porsche GT4 e-Performance

The **LWP3-GT4-SDK** is a high-performance C++20 library designed for researchers and developers building autonomous driving stacks. It transforms the LEGO Technic Porsche GT4 (42176) into a deterministic robotic platform.

***

### Disclaimer
*Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG. LEGO® is a trademark of the LEGO Group of companies. This project is not sponsored, authorized, or endorsed by the Porsche AG or LEGO Group. This SDK is an independent project developed for interoperability purposes under nominative fair use.*

***

## 🚀 Key Features
- **Latency Tracking:** Real-time P50/P99 latency profiling.
- **Lock-Free:** Atomic data latches for zero-contention state access.
- **Deterministic:** Nanosecond timestamps for every physical event.

## 📦 Installation
**Prerequisites:**
- Ubuntu 22.04+ (or any Linux with BlueZ 5.50+)
- `libsimpleble-dev`
- C++20 compatible compiler

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 💻 API Usage

The SDK follows a Pulse & Poll pattern. Your application owns the control loop (e.g., 50Hz) and the HAL handles the hardware communication.
Minimal Example
```C++

#include "Lwp3Gt4.hpp"

int main() {
    LWP3::PorscheGt4 car;
    
    // 1. Persistent Connect (waits for button press)
    car.connect("28:3C:90:9C:82:14");
    
    // 2. Calibrate hardware margins
    car.autoCalibrate();

    // 3. Autonomous Loop
    while(true) {
        // Poll latest state (lock-free)
        auto state = car.getLatestTelemetry();
        
        // Compute control (e.g., Sine wave)
        LWP3::Command cmd;
        cmd.throttle = 40;
        cmd.steer = static_cast<int>(50.0 * sin(t));

        // Pulse command to HAL
        car.sendCommand(cmd);
        
        std::this_thread::sleep_for(20ms);
    }
}
```

## ⚙️ Calibration Logic

The SDK performs a 5-step robust calibration to protect the steering servo:  
`Capture Initial -> Sweep Left -> Return Initial -> Sweep Right -> Return Zero.`

Soft margins are automatically calculated to stop steering 4 units before physical contact with the chassis.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

*Author: lsuciu70* *Version: 4.9.0 "The Deterministic HAL"*
