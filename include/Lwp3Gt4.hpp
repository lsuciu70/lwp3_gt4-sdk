#pragma once
#include <simpleble/SimpleBLE.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string_view>
#include <thread>
#include <vector>

namespace LWP3 {

using TimestampNs = uint64_t;

struct Command {
    int32_t steer = 0;
    int32_t throttle = 0;
};

struct Telemetry {
    int32_t steer_pos = 0;
    TimestampNs timestamp_ns = 0;
};

struct LatencyStats {
    float mean_ms;
    float p50_ms;
    float p99_ms;
};

class PorscheGt4 {
   public:
    PorscheGt4();
    ~PorscheGt4();

    bool connect(std::string_view address);
    void disconnect();
    bool autoCalibrate();

    void sendCommand(const Command& cmd) noexcept;
    Telemetry getLatestTelemetry() const noexcept;

    /** @brief Calculates Mean, P50, and P99 latency from recent samples. */
    LatencyStats getLatencyStats();

    bool isReady() const noexcept;

   private:
    mutable SimpleBLE::Peripheral porsche;
    std::jthread _txThread;
    std::atomic<bool> _running{false};
    std::atomic<bool> _isCalibrated{false};
    std::condition_variable _txCv;
    std::mutex _txMtx;

    std::atomic<Command> _latestCmd{Command{0, 0}};
    std::atomic<bool> _hasNewCmd{false};
    std::atomic<Telemetry> _telemetryLatch{Telemetry{}};

    std::atomic<int32_t> _rawSteerPos{0};
    std::atomic<bool> _telemetryActive{false};
    std::atomic<uint8_t> _virtualDrivePort{0xFF};

    // Profiling members
    std::mutex _statsMtx;
    std::vector<float> _latencySamples;
    std::atomic<TimestampNs> _lastTxTime{0};

    void txLoop(std::stop_token st);
    void setupHandshake();
    TimestampNs getNowNs() const;

    SimpleBLE::ByteArray buildSteerCmd(int32_t abs_angle);
    SimpleBLE::ByteArray buildDriveCmd(int8_t speed);
    int32_t sweep_to_limit(int8_t speed);
    void sendReliable(const SimpleBLE::ByteArray& data);

    int32_t hardwareCenter = 0;
};

}  // namespace LWP3
