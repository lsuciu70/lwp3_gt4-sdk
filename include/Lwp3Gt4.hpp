/**
 * @file Lwp3Gt4.hpp
 * @author lsuciu
 * @brief High-performance C++20 HAL for LEGO Technic Porsche GT4 (LWP3 Protocol).
 * @version 3.0.0
 * * This version introduces a fully asynchronous, triple-threaded architecture
 * designed for real-time ADAS and autonomous racing applications.
 */

#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <string_view>
#include <thread>

namespace LWP3 {

/**
 * @brief Thread-safe telemetry snapshot.
 * Bundles all physical hardware states into a single atomic push.
 */
struct Telemetry {
    int32_t steer_pos;      ///< Current relative steering angle (center = 0)
    uint8_t battery_level;  ///< Hub battery percentage (0-100)
    bool link_healthy;      ///< False if BLE watchdog or TX errors detected
    uint64_t timestamp_ns;  ///< Monotonic raw nanoseconds for state estimation
};

/**
 * @brief Internal Thread-Safe SPSC (Single-Producer Single-Consumer) Queue.
 * Bridges the high-speed BLE RX/TX threads with the Autonomy application loop.
 */
template <typename T>
class TSQueue {
   public:
    void push(const T& v) {
        std::lock_guard<std::mutex> lock(m_);
        q_.push_back(v);
    }

    bool try_pop(T& out) {
        std::lock_guard<std::mutex> lock(m_);
        if (q_.empty()) return false;
        out = q_.front();
        q_.pop_front();
        return true;
    }

   private:
    std::mutex m_;
    std::deque<T> q_;
};

/**
 * @brief Internal representation of a target control state.
 */
struct ControlState {
    int8_t throttle = 0;
    int32_t steer = 0;
};

/**
 * @brief Prepared Bluetooth byte packets for the hardware layer.
 */
struct CommandBatch {
    bool useVirtual = false;
    SimpleBLE::ByteArray virtualDrive;
    SimpleBLE::ByteArray leftMotor;
    SimpleBLE::ByteArray rightMotor;
    SimpleBLE::ByteArray steering;
};

/**
 * @class PorscheGt4
 * @brief The primary Hardware Abstraction Layer (HAL) for the vehicle.
 */
class PorscheGt4 {
   public:
    PorscheGt4();
    ~PorscheGt4();

    /**
     * @brief Connects to the vehicle via MAC address.
     * Spawns the internal TX and Control threads upon success.
     */
    bool connect(std::string_view address);

    /**
     * @brief Cleanly shuts down motors and joins threads.
     */
    void disconnect();

    /**
     * @brief Performs a physical calibration sweep.
     * Detects mechanical end-stops and calculates soft-margin limits.
     */
    bool autoCalibrate();

    /**
     * @brief Updates the vehicle targets (Non-blocking).
     * Commands are rate-limited and damped internally to protect gears.
     * @param throttle Speed -100 to 100.
     * @param steer Relative angle from center.
     */
    void setCommand(int8_t throttle, int32_t steer) noexcept;

    /**
     * @brief Retrieves the next telemetry packet from the queue.
     * @return true if data was available.
     */
    bool pollTelemetry(Telemetry& out_telemetry);

    /**
     * @brief Emergengy stop. Kills throttle but maintains current steering.
     */
    void stop() noexcept;

    /** @brief Simulation hook for software-in-the-loop testing. */
    void mockBleNotification(int32_t simulated_steer);

    // Status Accessors
    uint8_t getBatteryLevel() const noexcept;
    bool isConnected() noexcept;
    bool isLinkHealthy() const noexcept;

   private:
    SimpleBLE::Peripheral porsche;

    std::jthread _controlThread;
    std::jthread _txThread;

    std::atomic<bool> _running{false};
    std::atomic<bool> _linkHealthy{true};
    std::atomic<bool> _calibrating{false};

    std::atomic<uint64_t> _lastBrainPulse{0};
    std::atomic<uint64_t> _lastRxPulse{0};
    std::atomic<uint64_t> _lastCommandTimestamp{0};

    TSQueue<Telemetry> _telemetryQueue;

    std::optional<CommandBatch> _latestTxBatch;
    std::mutex _txMtx;
    std::condition_variable _txCv;
    std::atomic<uint64_t> _txSeq{0};
    int _txErrorCount{0};

    ControlState _targetState;
    std::mutex _targetMtx;

    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<uint8_t> batteryLevel{100};
    std::atomic<bool> telemetryActive{false};
    std::atomic<uint8_t> virtualDrivePort{0xFF};

    int32_t hardwareCenter = 0;
    int32_t minRelative = -65;
    int32_t maxRelative = 65;

    void controlLoop(std::stop_token st);
    void txLoop(std::stop_token st);
    void sendImmediate(const SimpleBLE::ByteArray& data);
    void sendReliable(const SimpleBLE::ByteArray& data);
    void setupHandshake();
    int32_t sweep_to_limit(int8_t speed, uint8_t power);
    void waitForMovement(int32_t target_absolute, int32_t tolerance = 4, int timeout_ms = 3000);

    SimpleBLE::ByteArray buildSteerCmd(int32_t absolute_angle, uint8_t speed = 50);
    SimpleBLE::ByteArray buildDriveCmd(uint8_t port, int8_t speed, uint8_t power = 100);
};
}  // namespace LWP3
