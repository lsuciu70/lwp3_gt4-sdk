/**
 * @file Lwp3Gt4.cpp
 * @brief Logic implementation for the Porsche GT4 v2.0 SDK.
 */

#include "Lwp3Gt4.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <ranges>

#include "Lwp3Constants.hpp"

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {}
PorscheGt4::~PorscheGt4() {
    disconnect();
}

/** @brief Returns current monotonic time in milliseconds. */
inline uint64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool PorscheGt4::connect(std::string_view address) {
    while (true) {
        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        auto adapter = adapters[0];
        std::cout << "\r[SDK] Searching for Porsche at " << address << "... " << std::flush;

        adapter.scan_for(1000);
        auto results = adapter.scan_get_results();
        auto it = std::ranges::find_if(results, [&](auto& p) { return p.address() == address; });

        if (it != results.end()) {
            porsche = *it;
            std::cout << "\n[SDK] Hub detected! Connecting..." << std::endl;

            try {
                porsche.connect();
                _running = true;
                _linkHealthy = true;
                _txErrorCount = 0;
                _calibrating = false;

                // Start asynchronous architecture
                _dispatchThread = std::jthread([this](std::stop_token st) { dispatchLoop(st); });
                _txThread = std::jthread([this](std::stop_token st) { txLoop(st); });

                setupHandshake();

                // Wait for sensors to come alive
                int retries = 0;
                while (!telemetryActive.load() && retries++ < 20) {
                    std::this_thread::sleep_for(50ms);
                }

                if (!telemetryActive.load()) {
                    std::cerr << "\n[SDK] Handshake timeout. Telemetry not active." << std::endl;
                    disconnect();
                    return false;
                }

                _controlThread = std::jthread([this](std::stop_token st) { controlLoop(st); });
                return true;
            } catch (const std::exception& e) {
                std::cerr << "\n[SDK] Connection failed: " << e.what() << ". Retrying..."
                          << std::endl;
            }
        }
        std::this_thread::sleep_for(500ms);
    }
}

/** * @brief Logic thread.
 * Converts high-level targets into motor commands at 50Hz.
 */
void PorscheGt4::controlLoop(std::stop_token st) {
    auto nextTick = std::chrono::steady_clock::now();

    while (!st.stop_requested()) {
        nextTick += 20ms;
        auto now = std::chrono::steady_clock::now();
        if (now > nextTick + 5ms) nextTick = now;

        if (_calibrating.load()) {
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        uint64_t current_time = now_ms();
        _linkHealthy = (_txErrorCount < 10);

        ControlState current;
        {
            std::lock_guard lock(_targetMtx);
            current = _targetState;
        }

        // Failsafe: Stop if brain stops sending pulses
        if (!_linkHealthy.load() || current_time - _lastBrainPulse.load() > 200) {
            current.throttle = 0;
            current.steer = 0;
        }

        int8_t speedL = std::clamp((int)-current.throttle, -100, 100);
        int8_t speedR = std::clamp((int)current.throttle, -100, 100);
        int32_t targetAbs = hardwareCenter + current.steer;

        CommandBatch batch;
        uint8_t vPort = virtualDrivePort.load();

        // Virtual Port Optimization (reduces BLE traffic)
        if (std::abs(speedL + speedR) <= 5 && vPort != 0xFF) {
            batch.useVirtual = true;
            batch.virtualDrive = buildDriveCmd(vPort, speedR);
        } else {
            batch.useVirtual = false;
            batch.leftMotor = buildDriveCmd(PORT_DRIVE_L, speedL);
            batch.rightMotor = buildDriveCmd(PORT_DRIVE_R, speedR);
        }
        batch.steering = buildSteerCmd(targetAbs);

        {
            std::lock_guard lock(_txMtx);
            _latestTxBatch = std::move(batch);
            _txSeq++;
        }
        _txCv.notify_one();

        std::this_thread::sleep_until(nextTick);
    }
}

/**
 * @brief Hardware pipe thread.
 * Consumes the latest available batch from the mailbox and pushes to Bluetooth.
 */
void PorscheGt4::txLoop(std::stop_token st) {
    uint64_t lastConsumedSeq = 0;

    while (!st.stop_requested()) {
        CommandBatch batch;
        {
            std::unique_lock lock(_txMtx);
            _txCv.wait(lock,
                       [&] { return _txSeq.load() != lastConsumedSeq || st.stop_requested(); });
            if (st.stop_requested()) return;

            lastConsumedSeq = _txSeq.load();
            batch = std::move(*_latestTxBatch);
        }

        try {
            if (batch.useVirtual) {
                porsche.write_command(SERVICE_UUID, CHAR_UUID, batch.virtualDrive);
            } else {
                porsche.write_command(SERVICE_UUID, CHAR_UUID, batch.leftMotor);
                porsche.write_command(SERVICE_UUID, CHAR_UUID, batch.rightMotor);
            }
            porsche.write_command(SERVICE_UUID, CHAR_UUID, batch.steering);
            _txErrorCount = 0;
        } catch (...) {
            _txErrorCount++;
        }
        std::this_thread::sleep_for(2ms);
    }
}

void PorscheGt4::dispatchLoop(std::stop_token st) {
    while (!st.stop_requested()) {
        std::function<void()> task;
        {
            std::unique_lock lock(_dispatchMtx);
            _dispatchCv.wait(lock, [&] { return !_dispatchQueue.empty() || st.stop_requested(); });
            if (st.stop_requested()) return;
            task = std::move(_dispatchQueue.front());
            _dispatchQueue.pop();
        }
        if (task) task();
    }
}

void PorscheGt4::updateControl(int8_t throttle, int32_t steer) noexcept {
    {
        std::lock_guard lock(_targetMtx);
        _targetState.throttle = throttle;
        _targetState.steer = std::clamp(steer, minRelative, maxRelative);
    }
    _lastCommandTimestamp = now_ms();
    _lastBrainPulse = _lastCommandTimestamp.load();
}

void PorscheGt4::queueTelemetry(std::function<void()> cb) {
    std::lock_guard lock(_dispatchMtx);
    if (_dispatchQueue.size() >= MAX_DISPATCH_QUEUE) _dispatchQueue.pop();
    _dispatchQueue.push(std::move(cb));
    _dispatchCv.notify_one();
}

void PorscheGt4::setupHandshake() {
    porsche.notify(SERVICE_UUID, CHAR_UUID, [this](SimpleBLE::ByteArray data) {
        if (!_running.load()) return;

        uint64_t current_ms = now_ms();
        _lastRxPulse = current_ms;

        auto* raw = reinterpret_cast<const uint8_t*>(data.data());
        if (data.size() < 3) return;

        // Steering Telemetry
        if (raw[2] == (uint8_t)MessageType::PORT_VALUE_SINGLE && raw[3] == PORT_STEER) {
            if (data.size() < 8) return;
            int32_t val;
            std::memcpy(&val, &raw[4], sizeof(int32_t));
            rawSteerPos.store(val);
            telemetryActive.store(true);

            if (onSteerUpdate) queueTelemetry([this, val] { onSteerUpdate(val - hardwareCenter); });

            uint64_t lastCmd = _lastCommandTimestamp.load();
            if (lastCmd > 0 && onLatencyProfileUpdate) {
                uint64_t latency = current_ms - lastCmd;
                queueTelemetry([this, latency] { onLatencyProfileUpdate(latency); });
            }
        }
        // Battery Telemetry
        else if (raw[2] == (uint8_t)MessageType::HUB_PROPERTIES &&
                 raw[4] == (uint8_t)HubPropertyOperation::UPDATE) {
            if (raw[3] == (uint8_t)HubProperty::BATTERY_VOLTAGE) {
                batteryLevel.store(raw[5]);
                if (onBatteryUpdate) queueTelemetry([this, b = raw[5]] { onBatteryUpdate(b); });
            }
        }
        // Virtual Port Mapping
        else if (raw[2] == (uint8_t)MessageType::HUB_ATTACHED_IO && raw[4] == 0x02) {
            if (raw[7] == PORT_DRIVE_L && raw[8] == PORT_DRIVE_R) virtualDrivePort.store(raw[3]);
        }
    });

    sendReliable({(uint8_t)MessageLength::PORT_INPUT_FORMAT, 0x00,
                  (uint8_t)MessageType::PORT_INPUT_FORMAT_SETUP, PORT_STEER, 0x02, 0x01, 0x00, 0x00,
                  0x00, 0x01});
    sendReliable({(uint8_t)MessageLength::HUB_PROP_SETUP, 0x00,
                  (uint8_t)MessageType::HUB_PROPERTIES, (uint8_t)HubProperty::BATTERY_VOLTAGE,
                  (uint8_t)HubPropertyOperation::ENABLE_UPDATES});
}

SimpleBLE::ByteArray PorscheGt4::buildDriveCmd(uint8_t port, int8_t speed, uint8_t power) {
    std::array<uint8_t, (uint8_t)MessageLength::MOTOR_POWER> buf = {
        (uint8_t)MessageLength::MOTOR_POWER,
        0x00,
        (uint8_t)MessageType::PORT_OUTPUT_COMMAND,
        port,
        0x11,
        (uint8_t)PortOutputSubCommand::START_POWER,
        (uint8_t)speed,
        power,
        0x03};
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), buf.size());
}

SimpleBLE::ByteArray PorscheGt4::buildSteerCmd(int32_t absolute_angle, uint8_t speed) {
    std::array<uint8_t, (uint8_t)MessageLength::GOTO_ABS_POS> buf = {
        (uint8_t)MessageLength::GOTO_ABS_POS,
        0x00,
        (uint8_t)MessageType::PORT_OUTPUT_COMMAND,
        PORT_STEER,
        0x11,
        (uint8_t)PortOutputSubCommand::GOTO_ABS_POS};
    std::memcpy(&buf[6], &absolute_angle, sizeof(int32_t));
    buf[10] = speed;
    buf[11] = 0x64;
    buf[12] = 0x7E;
    buf[13] = 0x03;
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), buf.size());
}

void PorscheGt4::sendImmediate(const SimpleBLE::ByteArray& data) {
    if (porsche.initialized() && porsche.is_connected()) {
        porsche.write_command(SERVICE_UUID, CHAR_UUID, data);
    }
}

void PorscheGt4::sendReliable(const SimpleBLE::ByteArray& data) {
    if (porsche.initialized() && porsche.is_connected()) {
        try {
            porsche.write_command(SERVICE_UUID, CHAR_UUID, data);
        } catch (...) {
        }
        std::this_thread::sleep_for(40ms);
    }
}

bool PorscheGt4::autoCalibrate() {
    for (int i = 0; i < 30 && !telemetryActive.load(); i++) std::this_thread::sleep_for(100ms);
    if (!telemetryActive.load()) return false;

    _calibrating.store(true);

    int32_t initialRawPos = rawSteerPos.load();
    hardwareCenter = initialRawPos;
    std::cout << "[SDK] Initial sensor position: " << hardwareCenter << std::endl;

    // 1. Go Left
    int32_t rawL = sweep_to_limit(-40, 60);
    std::cout << "[SDK] Left physical limit: " << rawL << std::endl;

    // 2. Return to Initial (un-wedge)
    sendReliable(buildSteerCmd(initialRawPos, 50));
    waitForMovement(initialRawPos, 4, 3000);
    std::this_thread::sleep_for(200ms);

    // 3. Go Right
    int32_t rawR = sweep_to_limit(40, 60);
    std::cout << "[SDK] Right physical limit: " << rawR << std::endl;

    _calibrating.store(false);

    if (std::abs(rawL - rawR) < 70) {
        std::cout << "[SDK] Range too small: " << std::abs(rawL - rawR) << " (Expected > 70)"
                  << std::endl;
        return false;
    }

    // Calculate True Center
    hardwareCenter = (rawL + rawR) / 2;
    minRelative = std::min(rawL, rawR) - hardwareCenter;
    maxRelative = std::max(rawL, rawR) - hardwareCenter;

    std::cout << "[SDK] True center calculated: " << hardwareCenter << std::endl;

    updateControl(0, 0);
    std::this_thread::sleep_for(500ms);

    _lastCommandTimestamp.store(0);
    return true;
}

int32_t PorscheGt4::sweep_to_limit(int8_t speed, uint8_t power) {
    sendReliable(buildDriveCmd(PORT_STEER, speed, power));
    std::this_thread::sleep_for(700ms);

    int32_t last_pos = rawSteerPos.load();
    int stuck_count = 0;

    for (int i = 0; i < 80; i++) {
        std::this_thread::sleep_for(80ms);
        int32_t p = rawSteerPos.load();
        if (std::abs(p - last_pos) <= 2)
            stuck_count++;
        else
            stuck_count = 0;
        if (stuck_count >= 4) break;
        last_pos = p;
    }

    sendReliable(buildDriveCmd(PORT_STEER, 0, 100));
    std::this_thread::sleep_for(300ms);
    return rawSteerPos.load();
}

void PorscheGt4::waitForMovement(int32_t target_absolute, int32_t tolerance, int timeout_ms) {
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        if (std::abs(rawSteerPos.load() - target_absolute) <= tolerance) return;
        std::this_thread::sleep_for(50ms);
        elapsed += 50;
    }
}

void PorscheGt4::disconnect() {
    _running = false;
    _controlThread.request_stop();
    _txThread.request_stop();
    _dispatchThread.request_stop();

    _dispatchCv.notify_all();
    _txCv.notify_all();

    if (porsche.initialized() && porsche.is_connected()) {
        uint8_t vPort = virtualDrivePort.load();
        if (vPort != 0xFF)
            sendReliable(buildDriveCmd(vPort, 0));
        else {
            sendReliable(buildDriveCmd(PORT_DRIVE_L, 0));
            sendReliable(buildDriveCmd(PORT_DRIVE_R, 0));
        }
        std::this_thread::sleep_for(200ms);
        porsche.disconnect();
    }
}

void PorscheGt4::stop() noexcept {
    updateControl(0, 0);
    _lastCommandTimestamp.store(0);
}

bool PorscheGt4::isConnected() noexcept {
    return porsche.initialized() && porsche.is_connected();
}
bool PorscheGt4::isLinkHealthy() const noexcept {
    return _linkHealthy.load();
}
uint8_t PorscheGt4::getBatteryLevel() const noexcept {
    return batteryLevel.load();
}

}  // namespace LWP3