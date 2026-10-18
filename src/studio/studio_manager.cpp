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
      _calibrator(nullptr),
      _state(StudioState::IDLE),
      _label(""),
      _durationSec(5.0f),
      _sessionId(""),
      _stepTimestampMs(0),
      _recordingStartTimeMs(0),
      _lastSampleTimeMs(0) {}

void StudioManager::begin(HealthKicksBleServer* bleServer, HapticDriver* haptic, ImuMpu6050* imu, ImuCalibrator* calibrator) {
    _bleServer = bleServer;
    _haptic = haptic;
    _imu = imu;
    _calibrator = calibrator;
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
            char* endPtr = nullptr;
            float parsed = strtof(durStr.c_str(), &endPtr);
            if (endPtr != durStr.c_str()) {
                dur = parsed;
            }
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
    if (_haptic) {
        _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 220, 200);
    }
}

void StudioManager::startRecording() {
    _frames.clear();
    size_t expectedSamples = (size_t)(_durationSec * IMU_SAMPLE_FREQ_HZ) + 10;
    _frames.reserve(expectedSamples);

    _state = StudioState::RECORDING;
    _recordingStartTimeMs = millis();
    _lastSampleTimeMs = 0;

    // Trigger clear start-of-recording haptic pulse
    if (_haptic) {
        _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 255, 350);
    }

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
                if (_haptic) {
                    _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 220, 200);
                }
            }
            break;

        case StudioState::COUNTDOWN_PULSE_2:
            if (now - _stepTimestampMs >= 1000) {
                _state = StudioState::COUNTDOWN_PULSE_3;
                _stepTimestampMs = now;
                Serial.println("[STUDIO] Countdown: 3/3");
                _bleServer->notifyStudioControl("COUNTDOWN 3/3");
                if (_haptic) {
                    _haptic->play(HAPTIC_PATTERN_CONTINUOUS, 220, 200);
                }
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
                    float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
                    float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;
                    if (_imu->readRawMetrics(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps)) {
                        if (_calibrator && _calibrator->isCalibrated()) {
                            _calibrator->applyCalibration(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
                        }

                        // Encode in standard network byte order (Big-Endian)
                        ImuRawFrame frame;
                        frame.delta_ms = htons((uint16_t)elapsed);
                        frame.ax = htons((int16_t)roundf(ax_g * 1000.0f));
                        frame.ay = htons((int16_t)roundf(ay_g * 1000.0f));
                        frame.az = htons((int16_t)roundf(az_g * 1000.0f));
                        frame.gx = htons((int16_t)roundf(gx_dps * 10.0f));
                        frame.gy = htons((int16_t)roundf(gy_dps * 10.0f));
                        frame.gz = htons((int16_t)roundf(gz_dps * 10.0f));

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

    // Trigger discrete double pulse to signal end of physical recording window
    if (_haptic) {
        _haptic->play(HAPTIC_PATTERN_DOUBLE_PULSE, 200, 250);
    }

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
    int rawFramesPerPacket = usable / IMU_BYTES_PER_FRAME;
    if (rawFramesPerPacket < 1) rawFramesPerPacket = 1;
    if (rawFramesPerPacket > 17) rawFramesPerPacket = 17;

    // Delta payload: 1 byte count + 14 bytes keyframe + K * 7 bytes deltas
    int maxDeltaFrames = (usable > 15) ? ((usable - 15) / 7) : 0;
    if (maxDeltaFrames > 32) maxDeltaFrames = 32; // Safe ceiling to fit in packetBuf

    size_t totalSamples = _frames.size();
    size_t offset = 0;
    uint16_t seqNum = 0;

    Serial.printf("[STUDIO] Starting Burst transmission: %u frames (MTU=%d, raw max=%d, delta max=%d)...\n",
                  (unsigned int)totalSamples, mtu, rawFramesPerPacket, maxDeltaFrames + 1);

    uint8_t packetBuf[260];

    while (offset < totalSamples) {
        // 1. Attempt lossless Run-Length Encoding (RLE) if at least 2 samples have identical sensor readings
        int rleCount = 0;
        uint8_t rleStepMs = 20;
        if ((totalSamples - offset) >= 2) {
            const ImuRawFrame& first = _frames[offset];
            const ImuRawFrame& second = _frames[offset + 1];
            int firstDiffMs = (int)ntohs(second.delta_ms) - (int)ntohs(first.delta_ms);

            if (firstDiffMs > 0 && firstDiffMs <= 255) {
                rleStepMs = (uint8_t)firstDiffMs;
                size_t maxPossibleRle = totalSamples - offset;
                if (maxPossibleRle > 255) maxPossibleRle = 255;

                size_t r = 1;
                while (r < maxPossibleRle) {
                    const ImuRawFrame& prevF = _frames[offset + r - 1];
                    const ImuRawFrame& currF = _frames[offset + r];

                    int curDiffMs = (int)ntohs(currF.delta_ms) - (int)ntohs(prevF.delta_ms);
                    if (curDiffMs != (int)rleStepMs) break;

                    if (currF.ax != first.ax || currF.ay != first.ay || currF.az != first.az ||
                        currF.gx != first.gx || currF.gy != first.gy || currF.gz != first.gz) {
                        break;
                    }
                    r++;
                }
                if (r >= 2) {
                    rleCount = (int)r;
                }
            }
        }

        if (rleCount >= 2) {
            const ImuRawFrame& first = _frames[offset];
            packetBuf[0] = BURST_PACKET_RLE_CHUNK; // 0x05
            packetBuf[1] = (uint8_t)((seqNum >> 8) & 0xFF);
            packetBuf[2] = (uint8_t)(seqNum & 0xFF);
            packetBuf[3] = 16; // Payload length: run_count (1) + start_delta_ms (2) + step_ms (1) + 6 axes (12) = 16 bytes

            packetBuf[4] = (uint8_t)rleCount;
            memcpy(&packetBuf[5], &first.delta_ms, 2);
            packetBuf[7] = rleStepMs;
            memcpy(&packetBuf[8], &first.ax, 12); // ax, ay, az, gx, gy, gz in big-endian network order

            _bleServer->sendBurstPacket(packetBuf, 4 + 16);
            offset += rleCount;
            seqNum++;
        } else {
            // 2. Attempt lossless delta compression if at least 2 samples remain and maxDeltaFrames >= 1
            int deltaCount = 0;
            if (maxDeltaFrames >= 1 && (totalSamples - offset) >= 2) {
                size_t maxPossibleDeltas = totalSamples - offset - 1;
                if (maxPossibleDeltas > (size_t)maxDeltaFrames) {
                    maxPossibleDeltas = (size_t)maxDeltaFrames;
                }

                for (size_t k = 0; k < maxPossibleDeltas; ++k) {
                    const ImuRawFrame& prev = _frames[offset + k];
                    const ImuRawFrame& curr = _frames[offset + k + 1];

                    int diff_ms = (int)ntohs(curr.delta_ms) - (int)ntohs(prev.delta_ms);
                    if (diff_ms < 0 || diff_ms > 255) break;

                    int diff_ax = (int)(int16_t)ntohs(curr.ax) - (int)(int16_t)ntohs(prev.ax);
                    if (diff_ax < -127 || diff_ax > 127) break;

                    int diff_ay = (int)(int16_t)ntohs(curr.ay) - (int)(int16_t)ntohs(prev.ay);
                    if (diff_ay < -127 || diff_ay > 127) break;

                    int diff_az = (int)(int16_t)ntohs(curr.az) - (int)(int16_t)ntohs(prev.az);
                    if (diff_az < -127 || diff_az > 127) break;

                    int diff_gx = (int)(int16_t)ntohs(curr.gx) - (int)(int16_t)ntohs(prev.gx);
                    if (diff_gx < -127 || diff_gx > 127) break;

                    int diff_gy = (int)(int16_t)ntohs(curr.gy) - (int)(int16_t)ntohs(prev.gy);
                    if (diff_gy < -127 || diff_gy > 127) break;

                    int diff_gz = (int)(int16_t)ntohs(curr.gz) - (int)(int16_t)ntohs(prev.gz);
                    if (diff_gz < -127 || diff_gz > 127) break;

                    deltaCount++;
                }
            }

            // If at least 1 delta frame fits, send DELTA_CHUNK (0x04)
            if (deltaCount >= 1) {
                uint8_t totalFramesInPacket = (uint8_t)(1 + deltaCount);
                uint8_t payloadBytes = (uint8_t)(1 + IMU_BYTES_PER_FRAME + deltaCount * 7);

                packetBuf[0] = BURST_PACKET_DELTA_CHUNK; // 0x04
                packetBuf[1] = (uint8_t)((seqNum >> 8) & 0xFF);
                packetBuf[2] = (uint8_t)(seqNum & 0xFF);
                packetBuf[3] = payloadBytes;

                packetBuf[4] = totalFramesInPacket;
                memcpy(&packetBuf[5], &_frames[offset], IMU_BYTES_PER_FRAME);

                size_t p = 5 + IMU_BYTES_PER_FRAME;
                for (int k = 0; k < deltaCount; ++k) {
                    const ImuRawFrame& prev = _frames[offset + k];
                    const ImuRawFrame& curr = _frames[offset + k + 1];

                    packetBuf[p + 0] = (uint8_t)((int)ntohs(curr.delta_ms) - (int)ntohs(prev.delta_ms));
                    packetBuf[p + 1] = (uint8_t)(int8_t)((int)(int16_t)ntohs(curr.ax) - (int)(int16_t)ntohs(prev.ax));
                    packetBuf[p + 2] = (uint8_t)(int8_t)((int)(int16_t)ntohs(curr.ay) - (int)(int16_t)ntohs(prev.ay));
                    packetBuf[p + 3] = (uint8_t)(int8_t)((int)(int16_t)ntohs(curr.az) - (int)(int16_t)ntohs(prev.az));
                    packetBuf[p + 4] = (uint8_t)(int8_t)((int)(int16_t)ntohs(curr.gx) - (int)(int16_t)ntohs(prev.gx));
                    packetBuf[p + 5] = (uint8_t)(int8_t)((int)(int16_t)ntohs(curr.gy) - (int)(int16_t)ntohs(prev.gy));
                    packetBuf[p + 6] = (uint8_t)(int8_t)((int)(int16_t)ntohs(curr.gz) - (int)(int16_t)ntohs(prev.gz));
                    p += 7;
                }

                _bleServer->sendBurstPacket(packetBuf, 4 + payloadBytes);
                offset += totalFramesInPacket;
                seqNum++;
            } else {
                // 3. Fallback to uncompressed DATA_CHUNK (0x02)
                size_t chunkCount = totalSamples - offset;
                if (chunkCount > (size_t)rawFramesPerPacket) {
                    chunkCount = rawFramesPerPacket;
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
            }
        }

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
