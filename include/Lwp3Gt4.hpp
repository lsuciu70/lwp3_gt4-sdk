/**
 * @file Lwp3Gt4.hpp
 * @author lsuciu
 * @brief v4.9 Deterministic HAL for LEGO Porsche GT4.
 * * This SDK follows the "Dumb HAL" philosophy: it provides a minimal latency path,
 * deterministic nanosecond timestamps, and hardware protection, while leaving
 * all control policy (smoothing, PIDs, trajectory) to the application layer.
 */

#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

namespace LWP3 {

/** @brief High-resolution monotonic timestamp in nanoseconds. */
using TimestampNs = uint64_t;

/**
 * @brief State snapshot of the vehicle.
 */
struct Telemetry {
    int32_t steer_pos;         ///< Relative steering position (center = 0).
    TimestampNs timestamp_ns;  ///< Arrival time of the packet (RX) via CLOCK_MONOTONIC_RAW.
    uint8_t battery_level;     ///< Hub battery percentage (0-100).
    bool link_healthy;         ///< Status of the BLE watchdog and connection.
};

/**
 * @brief Input command for the vehicle motors.
 */
struct Command {
    int steer;     ///< Target relative angle. Clamped by HAL soft-margins.
    int throttle;  ///< Target speed (-100 to 100).
};

/**
 * @brief Internal record of a transmitted command for latency profiling.
 */
struct TxEvent {
    int steer_cmd;             ///< The steering angle sent.
    TimestampNs timestamp_ns;  ///< The exact time the bytes were sent to the BLE radio.
};

/**
 * @brief Running performance metrics for the hardware link.
 */
struct LatencyStats {
    float mean_ms;  ///< Average round-trip time (TX to RX arrival).
    float p50_ms;   ///< Median latency.
    float p99_ms;   ///< Tail latency (99th percentile).
};

/**
 * @brief Configuration parameters for the HAL execution threads.
 */
struct HalConfig {
    int tx_rate_limit_ms = 15;      ///< Minimum delay between BLE writes (BlueZ stability).
    int watchdog_timeout_ms = 200;  ///< Cut throttle if no command received in this window.
    int keepalive_ms = 250;         ///< Frequency of heartbeat packets to the Hub.
};

/** @brief Connection status enum. */
enum class HalStatus { OK, DISCONNECTED, TIMEOUT, DEGRADED };

/**
 * @class PorscheGt4
 * @brief The Hardware Abstraction Layer for the LEGO Porsche GT4.
 * * Manages two primary threads:
 * 1. BLE RX (Internal to SimpleBLE): Ingests sensor data.
 * 2. TX Thread (Internal to HAL): Manages the command queue and hardware safety.
 */
class PorscheGt4 {
   public:
    PorscheGt4();
    ~PorscheGt4();

    /**
     * @brief Persistent Connection Loop.
     * * Scans for the specified MAC address indefinitely. Once found, it performs
     * GATT service discovery and verifies the LWP3 handshake before returning.
     * * @param address The Bluetooth MAC address of the LEGO Hub.
     * @return true if successfully connected and handshaked.
     */
    bool connect(std::string_view address);

    /**
     * @brief Gracefully shuts down motors and terminates background threads.
     */
    void disconnect();

    /**
     * @brief Multi-step robust physical calibration.
     * * Performs a 5-step process:
     * 1. Capture initial posture.
     * 2. Sweep to left physical limit.
     * 3. Gentle return to initial posture.
     * 4. Sweep to right physical limit.
     * 5. Calculate center and return to zero.
     * * @note Clamps future commands to detected soft-margins (limit - 4 units).
     * @return true if calibration succeeded and range is valid.
     */
    bool autoCalibrate();

    /**
     * @brief Updates HAL timing and watchdog parameters.
     */
    void configure(const HalConfig& cfg);

    /**
     * @brief Enqueues a movement command (Non-blocking).
     * * The command is sanitized and passed to the TX thread for immediate dispatch.
     * @param cmd Struct containing steer and throttle targets.
     */
    void sendCommand(const Command& cmd);

    /**
     * @brief Thread-safe, lock-free access to the most recent vehicle state.
     * @return The latest Telemetry packet received.
     */
    Telemetry getLatestTelemetry() const noexcept;

    /**
     * @brief Calculates latency statistics based on TX/RX time-of-flight.
     */
    LatencyStats getLatencyStats();

    /**
     * @brief Returns current operational health of the HAL.
     */
    HalStatus getStatus() const noexcept;

    /** @brief Asynchronous callback triggered on every telemetry arrival. */
    std::function<void(const Telemetry&)> onTelemetry;

   private:
    SimpleBLE::Peripheral porsche;
    HalConfig _config;

    std::jthread _txThread;
    std::atomic<bool> _running{false};
    std::atomic<bool> _isHardwareReady{false};

    mutable std::mutex _stateMtx;
    Telemetry _latestTelemetry{0, 0, 100, false};
    std::atomic<HalStatus> _status{HalStatus::DISCONNECTED};

    std::atomic<int32_t> rawSteerPos{0};
    std::atomic<bool> _calibrating{false};
    std::atomic<bool> _telemetryActive{false};

    std::deque<Command> _cmdQueue;
    std::mutex _queueMtx;
    std::condition_variable _queueCv;

    std::mutex _statsMtx;
    std::deque<TxEvent> _pendingTx;
    std::vector<float> _latencySamples;
    const size_t MAX_SAMPLES = 50;

    std::atomic<TimestampNs> _lastCmdTime{0};
    int32_t _lastSentSteer = 0;

    /** @brief Main TX dispatch loop. Handles watchdog and mechanical safety ramps. */
    void txLoop(std::stop_token st);

    /** @brief Subscribes to GATT notifications and requests LWP3 format. */
    void setupHandshake();

    /** @brief Internal handler for incoming BLE bytes. */
    void updateTelemetry(int32_t pos, uint8_t batt);

    /** @brief Fetches high-res time from CLOCK_MONOTONIC_RAW. */
    TimestampNs getNowNs() const;

    /** @brief Formats an LWP3 Absolute Position packet for the steering motor. */
    SimpleBLE::ByteArray buildSteerCmd(int32_t absolute_angle, uint8_t speed = 50);

    /** @brief Formats an LWP3 StartSpeed (0x01) or DirectWrite (0x51) drive packet. */
    SimpleBLE::ByteArray buildDriveCmd(uint8_t port, int8_t speed);

    /** @brief Open-loop motor drive used to find physical hard-stops. */
    int32_t sweep_to_limit(int8_t speed, std::string_view side);

    /** @brief Blocking write-and-sleep for calibration/setup reliability. */
    void sendReliable(const SimpleBLE::ByteArray& data);

    /** @brief Blocks until the steering rack reaches a specific absolute target. */
    void waitForMovement(int32_t target_abs, int timeout_ms = 2000);

    int32_t hardwareCenter = 0;
    int32_t minRelative = -65;
    int32_t maxRelative = 65;
    std::atomic<uint8_t> virtualDrivePort{0xFF};
};

}  // namespace LWP3
