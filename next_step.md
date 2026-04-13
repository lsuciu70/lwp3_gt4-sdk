# LWP3-GT4-SDK: v1.1 Development Roadmap

This document outlines the planned improvements and architectural refinements for the next major iteration of the Porsche GT4 SDK.

## 1. Professional CMake Deployment
Current implementation works as a direct source inclusion or git submodule. v1.1 will introduce standard library installation patterns.
* **Objective:** Allow the SDK to be installed system-wide.
* **Tasks:**
    * Add `install()` targets to `CMakeLists.txt` for headers and binaries.
    * Generate a `Lwp3Gt4Config.cmake` file to support `find_package(Lwp3Gt4)`.
    * Ensure proper export of the `LWP3::PorscheGt4` target.

## 2. Battery Telemetry Refinement
The current raw value reading will be abstracted into a meaningful metric for the end-user.
* **Objective:** Map raw voltage readings to an accurate charge percentage.
* **Tasks:**
    * Research the discharge curve for the Technic Hub (approx. 6.0V to 9.0V range).
    * Implement a mapping function (Linear or Lookup Table) to return a 0-100% value.
    * Add a `getVoltage()` method for users requiring raw millivolt data.

## 3. Structured Error Handling
Move away from `std::cerr` logs toward a programmable error-handling interface.
* **Objective:** Allow the client application to react to hardware or protocol failures.
* **Tasks:**
    * Implement an `onError` callback: `std::function<void(LWP3::ErrorCode, std::string)>`.
    * Define `ErrorCode` enum (e.g., `TIMEOUT`, `STALL_DETECTED`, `LINK_LOSS`, `COMMAND_REJECTED`).
    * Replace internal `std::cerr` calls with the new callback mechanism.

## 4. Emergency Stop Watchdog (New Feature)
A dedicated, on-demand safety thread to monitor the car's vitals and prevent "runaway" scenarios.
* **Objective:** Automatically halt the car if safety parameters are breached.
* **Tasks:**
    * Implement a `startSafetyWatchdog()` method that launches a background thread.
    * **RSSI Watchdog:** Trigger an emergency stop if signal strength drops below a threshold (e.g., -90 dBm).
    * **Heartbeat Watchdog:** Monitor for a lack of incoming telemetry packets.
    * **Collision/Stall:** Use motor feedback to detect if the car is physically stuck while driving.

## 5. Metadata & Versioning
* **Objective:** Formalize the project structure for public release.
* **Tasks:**
    * Add `version.hpp` with macro definitions for `LWP3_SDK_VERSION_MAJOR/MINOR`.
    * Implement a `getLibraryVersion()` static method.
