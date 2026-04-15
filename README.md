# LWP3-GT4-SDK (v4.9)
### Deterministic Hardware Abstraction Layer for LEGO Porsche GT4 Autonomous Racing

The **LWP3-GT4-SDK** is a high-performance C++20 library designed for researchers and developers building autonomous driving stacks. It transforms the LEGO Technic Porsche GT4 (42176) into a deterministic robotic platform.

***

### Disclaimer
*Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG. LEGO® is a trademark of the LEGO Group of companies. This project is not sponsored, authorized, or endorsed by the Porsche AG or LEGO Group. This SDK is an independent project developed for interoperability purposes under nominative fair use.*

***

## 🚀 Key Features
- **ADAS Ready:** Nanosecond-precision timestamps (`CLOCK_MONOTONIC_RAW`) for both RX and TX events.
- **Asynchronous Pipeline:** Dedicated high-speed TX thread (66Hz) and lock-free Telemetry access.
- **Pure HAL Philosophy:** No internal smoothing or PID control; the HAL provides raw, deterministic command relay.
- **Mechanical Bodyguard:** Built-in hardware protection ramps and soft-margin enforcement to protect plastic gears.
- **Persistent Connection:** Auto-discovery loop that waits for you to press the Hub button.

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

## 🛠 Architecture

1. **Application Layer**: Owns the Kalman Filter, Path Planning, and PID control.  
2. **HAL (This SDK)**: Provides a deterministic pipe to the hardware with gear protection.  
1. **LWP3 Protocol**: Low-level byte communication with the LEGO Hub.

## ⚙️ Calibration Logic

The SDK performs a 5-step robust calibration to protect the steering servo:  
`Capture Initial -> Sweep Left -> Return Initial -> Sweep Right -> Return Zero.`

Soft margins are automatically calculated to stop steering 4 units before physical contact with the chassis.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

*Author: lsuciu70* *Version: 4.9.0 "The Deterministic HAL"*
