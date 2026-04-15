#include "Lwp3Gt4.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>

#include "Lwp3Constants.hpp"

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {}
PorscheGt4::~PorscheGt4() {
    disconnect();
}

TimestampNs PorscheGt4::getNowNs() const {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<TimestampNs>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

void PorscheGt4::configure(const HalConfig& cfg) {
    _config = cfg;
}

bool PorscheGt4::connect(std::string_view address) {
    while (true) {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        auto adapter = adapters[0];
        std::cout << "\r[HAL] Waiting for Porsche (" << address << ")... " << std::flush;

        adapter.scan_for(1000);
        auto results = adapter.scan_get_results();
        auto it = std::ranges::find_if(results, [&](auto& p) { return p.address() == address; });

        if (it != results.end()) {
            porsche = *it;
            try {
                porsche.connect();

                // BlueZ Service Discovery: Loop until UUID handles are found
                bool discovered = false;
                for (int i = 0; i < 30; ++i) {
                    std::this_thread::sleep_for(100ms);
                    auto services = porsche.services();
                    for (auto& s : services) {
                        if (s.uuid() == SERVICE_UUID) {
                            discovered = true;
                            break;
                        }
                    }
                    if (discovered) break;
                }
                if (!discovered) {
                    porsche.disconnect();
                    continue;
                }

                setupHandshake();
                _running = true;
                _isHardwareReady = true;
                _status = HalStatus::OK;

                // Fire background TX thread only after radio is verified
                _txThread = std::jthread([this](std::stop_token st) { txLoop(st); });
                return true;
            } catch (...) {
                std::this_thread::sleep_for(1s);
            }
        }
    }
}

void PorscheGt4::sendCommand(const Command& cmd) {
    {
        std::lock_guard lock(_queueMtx);
        // HAL Layer: Clamping to detected physical soft-margins
        _cmdQueue.push_back(
            {std::clamp(cmd.steer, minRelative, maxRelative), std::clamp(cmd.throttle, -100, 100)});
    }
    _lastCmdTime = getNowNs();
    _queueCv.notify_one();
}

void PorscheGt4::txLoop(std::stop_token st) {
    Command target = {0, 0};
    while (!st.stop_requested()) {
        std::unique_lock lock(_queueMtx);
        // Wait for next pulse or keepalive timeout
        _queueCv.wait_for(lock, 250ms, [&] { return !_cmdQueue.empty() || st.stop_requested(); });
        if (st.stop_requested()) return;

        if (!_cmdQueue.empty()) {
            target = _cmdQueue.back();
            _cmdQueue.clear();
        }
        lock.unlock();

        if (_calibrating.load()) continue;

        // Safety Watchdog: Check if ADAS layer is still alive
        if ((getNowNs() - _lastCmdTime) > (uint64_t)_config.watchdog_timeout_ms * 1'000'000ULL) {
            target.throttle = 0;
            _status = HalStatus::TIMEOUT;
        } else {
            _status = HalStatus::OK;
        }

        // MECHANICAL SAFETY RAMP: Protects plastic gears from high-torque snaps
        // This is a hardware protection fuse, not a behavioral smoothing loop.
        int delta = target.steer - _lastSentSteer;
        const int max_step = 15;
        if (delta > max_step)
            _lastSentSteer += max_step;
        else if (delta < -max_step)
            _lastSentSteer -= max_step;
        else
            _lastSentSteer = target.steer;

        TimestampNs txTime = getNowNs();
        try {
            if (!_isHardwareReady.load() || !porsche.is_connected()) continue;

            uint8_t vPort = virtualDrivePort.load();
            if (vPort != 0xFF) {
                porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                      buildDriveCmd(vPort, target.throttle));
            } else {
                // Fallback: Individual motor drive if virtual port not handshaked
                porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                      buildDriveCmd(PORT_DRIVE_L, -target.throttle));
                porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                      buildDriveCmd(PORT_DRIVE_R, target.throttle));
            }
            porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                  buildSteerCmd(hardwareCenter + _lastSentSteer));

            // Record TX timestamp for ADAS state estimation
            std::lock_guard sLock(_statsMtx);
            _pendingTx.push_back({_lastSentSteer, txTime});
        } catch (...) {
            _status = HalStatus::DEGRADED;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(_config.tx_rate_limit_ms));
    }
}

bool PorscheGt4::autoCalibrate() {
    for (int i = 0; i < 30 && !_telemetryActive.load(); i++) std::this_thread::sleep_for(100ms);
    if (!_telemetryActive.load()) return false;
    _calibrating = true;
    int32_t initialRawPos = rawSteerPos.load();

    // 1. Sweep to left hard-stop
    int32_t rawL = sweep_to_limit(-15, "LEFT");

    // 2. Gentle Return to initial to reset gear tension
    sendReliable(buildSteerCmd(initialRawPos, 25));
    waitForMovement(initialRawPos, 3000);

    // 3. Sweep to right hard-stop
    int32_t rawR = sweep_to_limit(15, "RIGHT");

    // 4. Calculate True Center and Soft Margins
    hardwareCenter = (rawL + rawR) / 2;
    minRelative = (std::min(rawL, rawR) - hardwareCenter) + 4;
    maxRelative = (std::max(rawL, rawR) - hardwareCenter) - 4;

    // 5. Gentle return to Zero (Center)
    sendReliable(buildSteerCmd(hardwareCenter, 25));
    waitForMovement(hardwareCenter, 3000);

    _lastSentSteer = 0;
    _calibrating = false;
    return true;
}

int32_t PorscheGt4::sweep_to_limit(int8_t speed, std::string_view side) {
    // Command raw speed via StartSpeed(0x01)
    std::array<uint8_t, 7> buf = {
        0x07, 0x00, 0x81, PORT_STEER, 0x11, 0x01, static_cast<uint8_t>(speed)};
    sendReliable(SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 7));
    std::this_thread::sleep_for(400ms);

    int32_t last = -999, stuck = 0;
    for (int i = 0; i < 80; ++i) {
        std::this_thread::sleep_for(100ms);
        int32_t p = rawSteerPos.load();
        if (std::abs(p - last) <= 1)
            stuck++;
        else
            stuck = 0;
        if (stuck >= 4) break;  // Physical stop reached
        last = p;
    }
    // Cut power
    std::array<uint8_t, 7> stop_buf = {0x07, 0x00, 0x81, PORT_STEER, 0x11, 0x01, 0x00};
    sendReliable(SimpleBLE::ByteArray(reinterpret_cast<const char*>(stop_buf.data()), 7));
    std::this_thread::sleep_for(200ms);
    return rawSteerPos.load();
}

void PorscheGt4::waitForMovement(int32_t target_abs, int timeout_ms) {
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        if (std::abs(rawSteerPos.load() - target_abs) <= 3) return;
        std::this_thread::sleep_for(100ms);
        elapsed += 100;
    }
}

void PorscheGt4::sendReliable(const SimpleBLE::ByteArray& data) {
    if (!_isHardwareReady.load()) return;
    try {
        porsche.write_command(SERVICE_UUID, CHAR_UUID, data);
    } catch (...) {
    }
    std::this_thread::sleep_for(60ms);
}

void PorscheGt4::setupHandshake() {
    porsche.notify(SERVICE_UUID, CHAR_UUID, [this](SimpleBLE::ByteArray data) {
        auto* raw = reinterpret_cast<const uint8_t*>(data.data());
        if (data.size() < 8) return;
        // Handle Steering sensor notifications
        if (raw[2] == 0x45 && raw[3] == PORT_STEER) {
            _telemetryActive = true;
            int32_t val;
            std::memcpy(&val, &raw[4], sizeof(int32_t));
            updateTelemetry(val, _latestTelemetry.battery_level);
        } else if (raw[2] == 0x04 && raw[4] == 0x02) {
            if (raw[7] == PORT_DRIVE_L && raw[8] == PORT_DRIVE_R) virtualDrivePort.store(raw[3]);
        }
    });
    // Request Format: Abs Position, Mode 2
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x0A, 0x00, 0x41, PORT_STEER, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
}

void PorscheGt4::updateTelemetry(int32_t pos, uint8_t batt) {
    TimestampNs rxTime = getNowNs();
    rawSteerPos.store(pos);
    int rel = pos - hardwareCenter;
    {
        std::lock_guard lock(_stateMtx);
        _latestTelemetry = {rel, rxTime, batt, true};
    }

    // Latency matching logic
    std::lock_guard sLock(_statsMtx);
    auto it = _pendingTx.begin();
    while (it != _pendingTx.end()) {
        if (std::abs(it->steer_cmd - rel) <= 2) {
            float ms = (rxTime - it->timestamp_ns) / 1'000'000.0f;
            _latencySamples.push_back(ms);
            if (_latencySamples.size() > MAX_SAMPLES)
                _latencySamples.erase(_latencySamples.begin());
            it = _pendingTx.erase(it);
        } else {
            ++it;
        }
    }
    if (onTelemetry) onTelemetry(_latestTelemetry);
}

SimpleBLE::ByteArray PorscheGt4::buildDriveCmd(uint8_t port, int8_t speed) {
    std::array<uint8_t, 8> buf = {0x08, 0x00, 0x81, port,
                                  0x11, 0x51, 0x00, static_cast<uint8_t>(speed)};
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), buf.size());
}

SimpleBLE::ByteArray PorscheGt4::buildSteerCmd(int32_t absolute_angle, uint8_t speed) {
    std::array<uint8_t, 14> buf = {0x0E, 0x00, 0x81, PORT_STEER, 0x11, 0x0D};
    std::memcpy(&buf[6], &absolute_angle, sizeof(int32_t));
    buf[10] = speed;
    buf[11] = 0x64;
    buf[12] = 0x7E;
    buf[13] = 0x03;
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), buf.size());
}

void PorscheGt4::disconnect() {
    _running = false;
    _isHardwareReady = false;
    _txThread.request_stop();
    _queueCv.notify_all();
    if (porsche.is_connected()) porsche.disconnect();
    _status = HalStatus::DISCONNECTED;
}

Telemetry PorscheGt4::getLatestTelemetry() const noexcept {
    std::lock_guard lock(_stateMtx);
    return _latestTelemetry;
}
LatencyStats PorscheGt4::getLatencyStats() {
    std::lock_guard lock(_statsMtx);
    if (_latencySamples.empty()) return {0, 0, 0};
    float sum = 0;
    for (float s : _latencySamples) sum += s;
    return {sum / _latencySamples.size(), 0, 0};
}
HalStatus PorscheGt4::getStatus() const noexcept {
    return _status.load();
}

}  // namespace LWP3
