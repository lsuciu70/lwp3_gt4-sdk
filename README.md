# LWP3-GT4-SDK (v3.0.0)
### High-Performance C++20 SDK for LEGO Technic Porsche GT4 Autonomous Racing

The **LWP3-GT4-SDK** is a professional-grade Hardware Abstraction Layer (HAL) designed to bridge high-level autonomous driving stacks (Kalman Filters, Pure Pursuit, MPC) with the LEGO Powered Up (LWP3) protocol.

***

### Disclaimer
*LEGO® is a trademark of the LEGO Group of companies. Porsche®, GT4®, and e-Performance™ are trademarks of Porsche AG. This project is not sponsored, authorized, or endorsed by the LEGO Group or Porsche AG. This SDK is an independent community project developed for interoperability purposes under nominative fair use.*

***

## 🚀 Key Features
- **Async Pipeline:** Non-blocking API using lock-free Telemetry queues.
- **Real-Time Safety:** Triple-threaded design ensures consistent 50Hz control loops.
- **Hardware Bodyguard:** Built-in rate limiting (~5°/20ms) and soft-margin mechanical protection.
- **Deterministic Estimation:** High-resolution nanosecond timestamps for latency compensation.

## 🛠 Architecture Overview
The SDK operates across three isolated execution contexts:
1. **BLE RX (Event-Driven):** Instant ingestion of hardware notifications.
2. **Control Loop (Fixed 50Hz):** Dampens steering requests and checks safety watchdogs.
3. **BLE TX (Fixed 66Hz):** Dedicated hardware pipe to maximize LWP3 throughput.



## 📥 Installation
**Prerequisites:**
- Ubuntu 22.04+ / Debian 12+
- `libsimpleble-dev`
- C++20 compatible compiler (GCC 11+ or Clang 14+)

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 💻 Usage Example (Async Autonomy)

The v3.0 API follows a Pulse & Poll pattern. You must feed the vehicle at ~50Hz to keep the watchdog active.
```C++

#include "Lwp3Gt4.hpp"

LWP3::PorscheGt4 car;
car.connect("28:3C:90:9C:82:14");
car.autoCalibrate();

while(running) {
    // 1. Poll newest state
    LWP3::Telemetry t;
    while(car.pollTelemetry(t)) {
        // Use t.steer_pos and t.timestamp_ns for Kalman Filtering
    }

    // 2. Decide and Pulse (50Hz)
    car.setCommand(throttle, target_steer);
    
    std::this_thread::sleep_for(20ms);
}
```

## ⚠️ Safety Warning

This SDK provides high torque to the steering rack. Always run autoCalibrate() at the start of every session to ensure soft-margins are calculated. Failure to calibrate may result in stripped plastic gears.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

---
*Author: lsuciu70* *Version: 2.0.0 "The ADAS Refactor"*
