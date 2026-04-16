#include "Lwp3Gt4.hpp"

#include <algorithm>  // Required for sorting samples
#include <chrono>
#include <cstring>

#include "Lwp3Constants.hpp"

using namespace std::chrono_literals;

namespace LWP3 {

PorscheGt4::PorscheGt4() {
    _latencySamples.reserve(100);
}

PorscheGt4::~PorscheGt4() {
    disconnect();
}

TimestampNs PorscheGt4::getNowNs() const {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return static_cast<TimestampNs>(ts.tv_sec) * 1'000'000'000ULL + ts.tv_nsec;
}

bool PorscheGt4::connect(std::string_view address) {
    auto adapters = SimpleBLE::Adapter::get_adapters();
    if (adapters.empty()) return false;
    auto adapter = adapters[0];
    adapter.scan_for(1000);
    auto results = adapter.scan_get_results();

    bool found = false;
    for (auto& p : results) {
        if (p.address() == address) {
            porsche = p;
            found = true;
            break;
        }
    }

    if (found) {
        porsche.connect();
        std::this_thread::sleep_for(1s);
        setupHandshake();
        _running = true;
        _txThread = std::jthread([this](std::stop_token st) { txLoop(st); });
        return true;
    }
    return false;
}

void PorscheGt4::setupHandshake() {
    porsche.notify(SERVICE_UUID, CHAR_UUID, [this](SimpleBLE::ByteArray data) {
        auto* raw = reinterpret_cast<const uint8_t*>(data.data());

        if (data.size() >= 8 && raw[2] == 0x45 && raw[3] == PORT_STEER) {
            _telemetryActive = true;
            int32_t val;
            std::memcpy(&val, &raw[4], sizeof(int32_t));

            // Latency tracking: Measure time since last command sent
            TimestampNs now = getNowNs();
            TimestampNs lastTx = _lastTxTime.load();
            if (lastTx > 0) {
                std::lock_guard<std::mutex> lock(_statsMtx);
                float lat_ms = static_cast<float>(now - lastTx) / 1'000'000.0f;
                if (_latencySamples.size() >= 100) _latencySamples.erase(_latencySamples.begin());
                _latencySamples.push_back(lat_ms);
            }

            _rawSteerPos.store(val);
            _telemetryLatch.store({val - hardwareCenter, now});
        }

        if (raw[2] == 0x04 && raw[4] == 0x02) {
            _virtualDrivePort.store(raw[3]);
        }
    });

    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x0A, 0x00, 0x41, PORT_STEER, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01});
    std::this_thread::sleep_for(200ms);
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x06, 0x00, 0x61, 0x01, PORT_DRIVE_L, PORT_DRIVE_R});
    for (int i = 0; i < 20 && _virtualDrivePort.load() == 0xFF; ++i)
        std::this_thread::sleep_for(100ms);
}

void PorscheGt4::txLoop(std::stop_token st) {
    while (!st.stop_requested()) {
        std::unique_lock lock(_txMtx);
        _txCv.wait_for(lock, 20ms, [&] { return _hasNewCmd.load() || st.stop_requested(); });
        if (st.stop_requested()) return;

        Command target = _latestCmd.load();
        _hasNewCmd.store(false);
        lock.unlock();

        if (!_isCalibrated.load() || _virtualDrivePort.load() == 0xFF) continue;

        try {
            porsche.write_command(SERVICE_UUID, CHAR_UUID, buildDriveCmd(target.throttle));
            porsche.write_command(SERVICE_UUID, CHAR_UUID,
                                  buildSteerCmd(hardwareCenter + target.steer));
            _lastTxTime.store(getNowNs());  // Mark TX time for latency profiling
        } catch (...) {
        }
    }
}

void PorscheGt4::sendCommand(const Command& cmd) noexcept {
    _latestCmd.store(cmd);
    _hasNewCmd.store(true);
    _txCv.notify_one();
}

LatencyStats PorscheGt4::getLatencyStats() {
    std::lock_guard<std::mutex> lock(_statsMtx);
    if (_latencySamples.empty()) return {0.0f, 0.0f, 0.0f};

    std::vector<float> sorted = _latencySamples;
    std::sort(sorted.begin(), sorted.end());

    float sum = 0;
    for (float v : sorted) sum += v;

    return {sum / static_cast<float>(sorted.size()), sorted[sorted.size() / 2],
            sorted[static_cast<size_t>(sorted.size() * 0.99f)]};
}

SimpleBLE::ByteArray PorscheGt4::buildDriveCmd(int8_t speed) {
    std::array<uint8_t, 8> buf = {0x08, 0x00, 0x81, _virtualDrivePort.load(),
                                  0x11, 0x51, 0x00, static_cast<uint8_t>(speed)};
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 8);
}

SimpleBLE::ByteArray PorscheGt4::buildSteerCmd(int32_t abs_angle) {
    std::array<uint8_t, 14> buf = {0x0E, 0x00, 0x81, PORT_STEER, 0x11, 0x0D};
    std::memcpy(&buf[6], &abs_angle, sizeof(int32_t));
    buf[10] = 50;
    buf[11] = 100;
    buf[12] = 0x7E;
    buf[13] = 0x03;
    return SimpleBLE::ByteArray(reinterpret_cast<const char*>(buf.data()), 14);
}

bool PorscheGt4::autoCalibrate() {
    for (int i = 0; i < 30 && !_telemetryActive.load(); i++) std::this_thread::sleep_for(100ms);
    if (!_telemetryActive.load()) return false;

    int32_t init = _rawSteerPos.load();
    int32_t rawL = sweep_to_limit(-20);
    sendReliable(buildSteerCmd(init));
    int32_t rawR = sweep_to_limit(20);

    hardwareCenter = (rawL + rawR) / 2;
    sendReliable(buildSteerCmd(hardwareCenter));
    _isCalibrated.store(true);
    return true;
}

int32_t PorscheGt4::sweep_to_limit(int8_t speed) {
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x07, 0x00, 0x81, PORT_STEER, 0x11, 0x01, static_cast<uint8_t>(speed)});
    std::this_thread::sleep_for(1s);
    porsche.write_command(SERVICE_UUID, CHAR_UUID,
                          {0x07, 0x00, 0x81, PORT_STEER, 0x11, 0x01, 0x00});
    return _rawSteerPos.load();
}

void PorscheGt4::sendReliable(const SimpleBLE::ByteArray& data) {
    porsche.write_command(SERVICE_UUID, CHAR_UUID, data);
    std::this_thread::sleep_for(500ms);
}

Telemetry PorscheGt4::getLatestTelemetry() const noexcept {
    return _telemetryLatch.load();
}
bool PorscheGt4::isReady() const noexcept {
    return _isCalibrated.load() && _virtualDrivePort.load() != 0xFF;
}
void PorscheGt4::disconnect() {
    _running = false;
    porsche.disconnect();
}

}  // namespace LWP3
