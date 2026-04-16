# LWP3-GT4-SDK (v4.9)
### High-Performance robotic HAL for LEGO Porsche GT4 e-Performance

This library provides a high-performance Hardware Abstraction Layer (HAL) for the LEGO Technic Porsche GT4 (42176).

***

### Disclaimer
*Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG. LEGO® is a trademark of the LEGO Group of companies. This project is not sponsored, authorized, or endorsed by the Porsche AG or LEGO Group. This SDK is an independent project developed for interoperability purposes under nominative fair use.*

***

## 🚀 Key Features
- **Latency Tracking:** Real-time P50/P99 latency profiling.
- **Lock-Free:** Atomic data latches for zero-contention state access.
- **Deterministic Timing**: Uses `CLOCK_MONOTONIC_RAW` for all profiling.
- **Non-Blocking Logic**: TX thread uses a rate-gate instead of sleep_for, maximizing responsiveness.
- **Epsilon Matching**: Latency is only recorded when the mechanical rack physically reaches the target threshold (ϵ=3.0∘).

## Performance Specs (Verified v7.0)

The following metrics were captured during the final release profile:
- **Mean System Latency**: 93.53 ms (Total mechanical response time).
- **Control Frequency**: 50 Hz (20 ms nominal cycles).
- **Jitter (P99)**: 258 ms (Accounts for mechanical settling and gear lash).

## Architecture: Two-Packet Synchronized Dispatch

To bypass the limitations of the Move Hub firmware and ensure D-Bus stability, the HAL uses a dual-packet architecture:
- **Virtual Port (0x61)**: Dynamically bonds physical Ports 50 (L) and 51 (R) for hardware-level drive synchronization.
- **Physical Port 52**: Independent absolute position control for high-precision steering.

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

### Slalom Example  
**Note**: Press the Hub button just before start

```C++

#include <cmath>
#include <iostream>
#include <thread>

#include "Lwp3Gt4.hpp"

int main() {
    LWP3::PorscheGt4 car;
    // Put your Lego Hub MAC here
    if (!car.connect("28:3C:90:9C:82:14")) {
        return 1;
    }

    std::cout << "Calibrating..." << std::endl;
    // Calibration is mandatory
    car.autoCalibrate();

    // Final check for the Virtual Port before slalom
    if (!car.isReady()) {
        std::cerr << "Hub failed to initialize virtual drive port!" << std::endl;
        return 1;
    }

    std::cout << "Starting Slalom..." << std::endl;
    for (int i = 0; i < 400; ++i) {
        float t = i * 0.02f;
        int32_t steer = static_cast<int32_t>(45.0f * std::sin(t * 3.5f));

        car.sendCommand({steer, 35});
        std::this_thread::sleep_for(20ms);
    }

    car.sendCommand({0, 0});
    std::this_thread::sleep_for(500ms);
    car.disconnect();
    return 0;
}
```

## ⚙️ Calibration Logic

The SDK performs a 5-step robust calibration to protect the steering servo:  
`Capture Initial -> Sweep Left -> Return Initial -> Sweep Right -> Return Zero.`  
Soft margins are automatically calculated to stop steering 4 units before physical contact with the chassis.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

*Author: lsuciu70* *Version: 7.0.0 "The Mechanical Truth"*
