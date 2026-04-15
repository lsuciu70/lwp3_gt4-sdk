/**
 * @file Lwp3Gt4.hpp
 * @author lsuciu
 * @brief High-performance ADAS SDK for LEGO Porsche GT4.
 * * v2.0 Features:
 * - Asynchronous Control Loop (50Hz)
 * - Mailbox-based TX Pacing (2ms)
 * - Mechanical-aware Calibration
 * - Thread-safe Telemetry Dispatching
 */

#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <string_view>
#include <thread>

namespace LWP3 {

/** @brief Internal representation of a target control state. */
struct ControlState {
    int8_t throttle = 0;  // -100 to 100
    int32_t steer = 0;    // Relative angle from center
};

/** @brief Prepared Bluetooth byte packets for the hardware. */
struct CommandBatch {
    bool useVirtual = false;
    SimpleBLE::ByteArray virtualDrive;
    SimpleBLE::ByteArray leftMotor;
    SimpleBLE::ByteArray rightMotor;
    SimpleBLE::ByteArray steering;
};

class PorscheGt4 {
   public:
    // --- Callbacks (Telemetry) ---
    std::function<void(int32_t)> onSteerUpdate;            ///< Notifies on steering angle change
    std::function<void(uint8_t)> onBatteryUpdate;          ///< Notifies on battery % change
    std::function<void(uint64_t)> onLatencyProfileUpdate;  ///< Notifies on end-to-end loop latency

    PorscheGt4();
    ~PorscheGt4();

    /** @brief Continuously scans and connects to a specific MAC address. */
    bool connect(std::string_view address);

    /** @brief Safely shuts down motors and threads before disconnecting. */
    void disconnect();

    /** @brief Executes a 5-step mechanical calibration to find true steering center. */
    bool autoCalibrate();

    /** @brief Updates the target driving state (Thread-safe). */
    void updateControl(int8_t throttle, int32_t steer) noexcept;

    /** @brief Instantly sets throttle and steering targets to zero. */
    void stop() noexcept;

    // --- Status Accessors ---
    uint8_t getBatteryLevel() const noexcept;
    bool isConnected() noexcept;
    bool isLinkHealthy() const noexcept;

   private:
    SimpleBLE::Peripheral porsche;

    // --- Threading Model ---
    std::jthread _controlThread;   ///< Logic loop (50Hz)
    std::jthread _txThread;        ///< Hardware pipe (2ms)
    std::jthread _dispatchThread;  ///< Telemetry callback isolation

    std::atomic<bool> _running{false};
    std::atomic<bool> _linkHealthy{true};
    std::atomic<bool> _calibrating{false};

    // --- Watchdog & Timestamps ---
    std::atomic<uint64_t> _lastBrainPulse{0};
    std::atomic<uint64_t> _lastRxPulse{0};
    std::atomic<uint64_t> _lastCommandTimestamp{0};

    // --- Thread Safety / Sync ---
    static constexpr size_t MAX_DISPATCH_QUEUE = 64;
    std::queue<std::function<void()>> _dispatchQueue;
    std::mutex _dispatchMtx;
    std::condition_variable _dispatchCv;

    std::optional<CommandBatch> _latestTxBatch;
    std::mutex _txMtx;
    std::condition_variable _txCv;
    std::atomic<uint64_t> _txSeq{0};
    int _txErrorCount{0};

    ControlState _targetState;
    std::mutex _targetMtx;

    // --- Hardware State ---
    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<uint8_t> batteryLevel{0};
    std::atomic<bool> telemetryActive{false};
    std::atomic<uint8_t> virtualDrivePort{0xFF};

    int32_t hardwareCenter = 0;
    int32_t minRelative = -65;
    int32_t maxRelative = 65;

    // --- Internal Loops ---
    void controlLoop(std::stop_token st);
    void txLoop(std::stop_token st);
    void dispatchLoop(std::stop_token st);

    void queueTelemetry(std::function<void()> cb);
    void sendImmediate(const SimpleBLE::ByteArray& data);
    void sendReliable(const SimpleBLE::ByteArray& data);

    void setupHandshake();
    int32_t sweep_to_limit(int8_t speed, uint8_t power);
    void waitForMovement(int32_t target_absolute, int32_t tolerance = 4, int timeout_ms = 3000);

    SimpleBLE::ByteArray buildSteerCmd(int32_t absolute_angle, uint8_t speed = 50);
    SimpleBLE::ByteArray buildDriveCmd(uint8_t port, int8_t speed, uint8_t power = 100);
};
}  // namespace LWP3