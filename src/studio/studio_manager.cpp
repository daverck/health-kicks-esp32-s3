#include "studio_manager.h"
#include <sstream>

static uint32_t computeCrc32(const uint8_t* data, size_t length) {
    static uint32_t table[256];
    static bool table_init = false;
    if (!table_init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int j = 0; j < 8; j++) {
                c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
            }
            table[i] = c;
        }
        table_init = true;
    }

    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; i++) {
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFU;
}

StudioManager::StudioManager()
    : _bleServer(nullptr),
      _haptic(nullptr),
      _imu(nullptr),
      _state(StudioState::IDLE),
      _label(""),
      _durationSec(5.0f),
      _sessionId(""),
      _stepTimestampMs(0),
      _recordingStartTimeMs(0),
      _lastSampleTimeMs(0) {}

void StudioManager::begin(HealthKicksBleServer* bleServer, HapticDriver* haptic, ImuMpu6050* imu) {
    _bleServer = bleServer;
    _haptic = haptic;
    _imu = imu;
    _state = StudioState::IDLE;
    _frames.reserve(1600); // Reserve for 30s at 50Hz (1500 frames)
}

void StudioManager::handleCommand(const std::string& command) {
    Serial.printf("[STUDIO] Command received: \"%s\"\n", command.c_str());

    if (command.rfind("START", 0) == 0) {
        std::istringstream iss(command);
        std::string startWord, label, durStr, sessId;
        iss >> startWord >> label >> durStr >> sessId;

        float dur = 5.0f;
        if (!durStr.empty()) {
            dur = std::stof(durStr);
        }
        if (dur < 1.0f) dur = 1.0f;
        if (dur > 30.0f) dur = 30.0f;
        if (label.empty()) label = "unlabeled";
        if (sessId.empty()) sessId = "default-session-id";

        if (_state != StudioState::IDLE) {
            if (sessId == _sessionId) {
                Serial.println("[STUDIO] Duplicate START command for same session: ignored.");
                return;
            }
            Serial.println("[STUDIO] Rejected: session already in progress.");
            _bleServer->notifyStudioControl("ERROR busy");
            return;
        }

        _label = label;
        _durationSec = dur;
        _sessionId = sessId;

        startCountdown();
    } else if (command == "CANCEL") {
        cancel();
    }
}

void StudioManager::startCountdown() {
    Serial.printf("[STUDIO] Starting countdown (3 haptic pulses) for session \"%s\" (%.1fs)...\n",
                  _sessionId.c_str(), _durationSec);

    _state = StudioState::COUNTDOWN_PULSE_1;
    _stepTimestampMs = millis();
    Serial.println("[STUDIO] Countdown: 1/3");
    _bleServer->notifyStudioControl("COUNTDOWN 1/3");
    _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 180, 100);
}

void StudioManager::startRecording() {
    _frames.clear();
    size_t expectedSamples = (size_t)(_durationSec * IMU_SAMPLE_FREQ_HZ) + 10;
    _frames.reserve(expectedSamples);

    _state = StudioState::RECORDING;
    _recordingStartTimeMs = millis();
    _lastSampleTimeMs = 0;

    char buf[64];
    snprintf(buf, sizeof(buf), "RECORDING %.1f", _durationSec);
    _bleServer->notifyStudioControl(buf);
    Serial.printf("[STUDIO] IMU recording started at %d Hz for %.1f seconds\n", IMU_SAMPLE_FREQ_HZ, _durationSec);
}

void StudioManager::update() {
    uint32_t now = millis();

    switch (_state) {
        case StudioState::IDLE:
            break;

        case StudioState::COUNTDOWN_PULSE_1:
            if (now - _stepTimestampMs >= 1000) {
                _state = StudioState::COUNTDOWN_PULSE_2;
                _stepTimestampMs = now;
                Serial.println("[STUDIO] Countdown: 2/3");
                _bleServer->notifyStudioControl("COUNTDOWN 2/3");
                _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 180, 100);
            }
            break;

        case StudioState::COUNTDOWN_PULSE_2:
            if (now - _stepTimestampMs >= 1000) {
                _state = StudioState::COUNTDOWN_PULSE_3;
                _stepTimestampMs = now;
                Serial.println("[STUDIO] Countdown: 3/3");
                _bleServer->notifyStudioControl("COUNTDOWN 3/3");
                _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 180, 100);
            }
            break;

        case StudioState::COUNTDOWN_PULSE_3:
            if (now - _stepTimestampMs >= 1000) {
                Serial.println("[STUDIO] Countdown finished -> Starting RECORDING");
                startRecording();
            }
            break;

        case StudioState::RECORDING: {
            uint32_t elapsed = now - _recordingStartTimeMs;
            uint32_t targetDurationMs = (uint32_t)(_durationSec * 1000.0f);

            if (elapsed >= targetDurationMs) {
                finishRecordingAndStreamBurst();
            } else if (now - _lastSampleTimeMs >= IMU_SAMPLE_PERIOD_MS) {
                _lastSampleTimeMs = now;
                if (_imu) {
                    ImuRawFrame frame;
                    if (_imu->readFrame(frame, (uint16_t)elapsed)) {
                        _frames.push_back(frame);
                    }
                }
            }
            break;
        }

        case StudioState::TRANSMITTING_BURST:
            break;
    }
}

void StudioManager::finishRecordingAndStreamBurst() {
    _state = StudioState::TRANSMITTING_BURST;

    Serial.printf("[STUDIO] Recording finished. Samples collected: %u\n", (unsigned int)_frames.size());

    char finishMsg[128];
    snprintf(finishMsg, sizeof(finishMsg), "FINISHED %u %s", (unsigned int)_frames.size(), _sessionId.c_str());
    _bleServer->notifyStudioControl(finishMsg);

    sendBurstPackets();

    _state = StudioState::IDLE;
    _frames.clear();
}

void StudioManager::sendBurstPackets() {
    if (!_bleServer || !_bleServer->isConnected()) {
        Serial.println("[STUDIO] Aborting burst: BLE client disconnected.");
        return;
    }

    uint16_t mtu = _bleServer->getNegotiatedMtu();
    int usable = mtu - 3 - 4; // MTU - 3 ATT - 4 Header
    int framesPerPacket = usable / IMU_BYTES_PER_FRAME;
    if (framesPerPacket < 1) framesPerPacket = 1;
    if (framesPerPacket > 17) framesPerPacket = 17;

    size_t totalSamples = _frames.size();
    size_t offset = 0;
    uint16_t seqNum = 0;

    Serial.printf("[STUDIO] Starting Burst transmission: %u frames (MTU=%d, %d frames/packet)...\n",
                  (unsigned int)totalSamples, mtu, framesPerPacket);

    uint8_t packetBuf[256];

    while (offset < totalSamples) {
        size_t chunkCount = totalSamples - offset;
        if (chunkCount > (size_t)framesPerPacket) {
            chunkCount = framesPerPacket;
        }

        uint8_t payloadBytes = (uint8_t)(chunkCount * IMU_BYTES_PER_FRAME);

        packetBuf[0] = BURST_PACKET_DATA_CHUNK; // 0x02
        packetBuf[1] = (uint8_t)((seqNum >> 8) & 0xFF);
        packetBuf[2] = (uint8_t)(seqNum & 0xFF);
        packetBuf[3] = payloadBytes;

        memcpy(&packetBuf[4], &_frames[offset], payloadBytes);

        _bleServer->sendBurstPacket(packetBuf, 4 + payloadBytes);

        offset += chunkCount;
        seqNum++;

        delay(5); // Inter-packet delay to prevent BLE radio buffer overflow
    }

    // Compute CRC32 IEEE 802.3 over all payload bytes transmitted
    uint32_t crc32 = computeCrc32(reinterpret_cast<const uint8_t*>(_frames.data()), totalSamples * IMU_BYTES_PER_FRAME);

    // END_OF_BURST packet (12 bytes)
    uint8_t endBuf[12];
    endBuf[0] = BURST_PACKET_END_OF_BURST; // 0x03
    endBuf[1] = (uint8_t)((seqNum >> 8) & 0xFF);
    endBuf[2] = (uint8_t)(seqNum & 0xFF);
    endBuf[3] = 0x00;

    endBuf[4] = (uint8_t)((totalSamples >> 24) & 0xFF);
    endBuf[5] = (uint8_t)((totalSamples >> 16) & 0xFF);
    endBuf[6] = (uint8_t)((totalSamples >> 8) & 0xFF);
    endBuf[7] = (uint8_t)(totalSamples & 0xFF);

    endBuf[8] = (uint8_t)((crc32 >> 24) & 0xFF);
    endBuf[9] = (uint8_t)((crc32 >> 16) & 0xFF);
    endBuf[10] = (uint8_t)((crc32 >> 8) & 0xFF);
    endBuf[11] = (uint8_t)(crc32 & 0xFF);

    _bleServer->sendBurstPacket(endBuf, sizeof(endBuf));

    Serial.printf("[STUDIO] Burst successfully transmitted: %u samples across %u packets (CRC32: 0x%08X)\n",
                  (unsigned int)totalSamples, seqNum, crc32);
}

void StudioManager::cancel() {
    if (_state != StudioState::IDLE) {
        _state = StudioState::IDLE;
        _frames.clear();
        _haptic->stop();
        _bleServer->notifyStudioControl("CANCELLED");
        Serial.println("[STUDIO] Session cancelled.");
    }
}
