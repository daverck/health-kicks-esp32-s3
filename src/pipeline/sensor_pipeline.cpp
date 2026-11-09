#include "pipeline/sensor_pipeline.h"
#include "power_manager.h"
#include <cmath>

SensorPipeline::SensorPipeline(
    ImuMpu6050& imu,
    ImuCalibrator& imuCalibrator,
    StepDetector& stepDetector,
    InactivityMonitor& inactivityMonitor,
    ActivityDetector& activityDetector,
    HealthKicksBleServer& bleServer,
    HapticDriver& haptic,
    StudioManager& studioManager,
    OfflineFallStorage& offlineFallStorage
) : imu(imu),
    imuCalibrator(imuCalibrator),
    stepDetector(stepDetector),
    inactivityMonitor(inactivityMonitor),
    activityDetector(activityDetector),
    bleServer(bleServer),
    haptic(haptic),
    studioManager(studioManager),
    offlineFallStorage(offlineFallStorage) {}

void SensorPipeline::onBleConnected(uint32_t nowMs) {
    StepCounterPayload payload = stepDetector.getPayload(nowMs);
    bleServer.notifyStepCounter(payload);
    lastNotifiedTotalSteps = stepDetector.getTotalSteps();
    hasPendingIdleConfirmation = false;
    Serial.printf("[STEP] BLE connected, sent initial step state: Total=%u\n", lastNotifiedTotalSteps);
}

void SensorPipeline::process(uint32_t nowMs, bool imuReady) {
    // Nominal IMU acquisition & Edge AI Activity Detection (only if Studio is not recording)
    if (!studioManager.isRecording() && (nowMs - lastImuReadMs >= IMU_SAMPLE_PERIOD_MS)) {
        lastImuReadMs = nowMs;

        if (imuReady) {
            float ax, ay, az, gx, gy, gz;
            if (imu.readRawMetrics(ax, ay, az, gx, gy, gz)) {
                // Ingest raw physical readings into dynamic orientation calibrator
                imuCalibrator.update(ax, ay, az, gx, gy, gz);

                // Apply dynamic alignment rotation matrix (corrects sensor PCB tilt)
                imuCalibrator.applyCalibration(ax, ay, az, gx, gy, gz);

                // Step detection on calibrated vertical acceleration with activity gating & GCT estimation
                bool stepDetected = stepDetector.processSample(ax, ay, az, gy, currentActivityState, nowMs);
                if (stepDetected) {
                    lastStepDetectedMs = nowMs;
                    hasPendingIdleConfirmation = true;
                    PowerManager::recordActivity(nowMs);

                    uint32_t totalSteps = stepDetector.getTotalSteps();
                    if (bleServer.isConnected() && (totalSteps - lastNotifiedTotalSteps >= STEP_NOTIFY_BATCH_COUNT)) {
                        StepCounterPayload payload = stepDetector.getPayload(nowMs);
                        bleServer.notifyStepCounter(payload);
                        lastNotifiedTotalSteps = totalSteps;
                        Serial.printf("[STEP] Batched notification (%u steps): Total=%u (W:%u, R:%u, S:%u, U:%u) | Cadence: %u SPM | GCT: %u ms\n",
                                      STEP_NOTIFY_BATCH_COUNT, totalSteps, stepDetector.getWalkSteps(),
                                      stepDetector.getRunSteps(), stepDetector.getStairsSteps(),
                                      stepDetector.getUnclassifiedSteps(), payload.cadence_spm, stepDetector.getAverageGctMs());
                    }
                }

                // Inactivity timer is reset only when steps occur in an active locomotion state.
                inactivityMonitor.processStepActivity(stepDetector.getTotalSteps(), currentActivityState, nowMs);

                // Process HITL ring buffer and pending dispatch
                processHitlBuffer(ax, ay, az, gx, gy, gz, nowMs);

                // Push calibrated sample into sliding buffer
                activityDetector.pushSample(ax, ay, az, gx, gy, gz);

                // Run real-time edge inference if step interval is due
                evaluateEdgeAi(nowMs);
            }
        }
    }

    // Rest / Idle confirmation notification check
    checkIdleRestConfirmation(nowMs);
}

void SensorPipeline::processHitlBuffer(float ax, float ay, float az, float gx, float gy, float gz, uint32_t nowMs) {
    // Store calibrated sample into HITL ring buffer (4.0s history)
    // Encode in standard network byte order (Big-Endian / htons) for consistency with Studio recordings
    ImuRawFrame hitlFrame;
    hitlFrame.delta_ms = 0;
    hitlFrame.ax = htons((int16_t)roundf(ax * 1000.0f));
    hitlFrame.ay = htons((int16_t)roundf(ay * 1000.0f));
    hitlFrame.az = htons((int16_t)roundf(az * 1000.0f));
    hitlFrame.gx = htons((int16_t)roundf(gx * 10.0f));
    hitlFrame.gy = htons((int16_t)roundf(gy * 10.0f));
    hitlFrame.gz = htons((int16_t)roundf(gz * 10.0f));

    hitlRingBuffer[hitlRingHead] = hitlFrame;
    hitlRingHead = (hitlRingHead + 1) % HITL_BUFFER_CAPACITY;
    if (hitlBufferCount < HITL_BUFFER_CAPACITY) {
        hitlBufferCount++;
    }

    if (hitlCapturePending) {
        if (hitlPostSamplesRemaining > 0) {
            hitlPostSamplesRemaining--;
        }
        if (hitlPostSamplesRemaining == 0) {
            hitlCapturePending = false;
            if (bleServer.isConnected() && bleServer.isHitlEnabled() && !studioManager.isBusy()) {
                std::vector<ImuRawFrame> burstFrames;
                burstFrames.reserve(hitlBufferCount);
                size_t startIdx = (hitlBufferCount == HITL_BUFFER_CAPACITY) ? hitlRingHead : 0;
                for (size_t i = 0; i < hitlBufferCount; ++i) {
                    size_t idx = (startIdx + i) % HITL_BUFFER_CAPACITY;
                    ImuRawFrame f = hitlRingBuffer[idx];
                    f.delta_ms = htons((uint16_t)(i * IMU_SAMPLE_PERIOD_MS));
                    burstFrames.push_back(f);
                }
                char sessionId[64];
                snprintf(sessionId, sizeof(sessionId), "hitl-%u", (unsigned int)hitlTriggerEpochSec);
                Serial.printf("[HITL] Dispatching 4.0s burst of %u samples for session %s...\n",
                              (unsigned int)burstFrames.size(), sessionId);
                studioManager.streamHitlBurst(burstFrames, sessionId);
            }
        }
    }
}

void SensorPipeline::evaluateEdgeAi(uint32_t nowMs) {
    if (!activityDetector.isEvaluationDue()) {
        return;
    }

    ActivityDetectionResult result;
    if (!activityDetector.detect(result)) {
        return;
    }

    if (result.confidence >= activityDetector.getConfidenceThreshold()) {
        uint32_t epochSec = (uint32_t)(nowMs / 1000);
        currentActivityState = result.stateCode;

        // Notify calibrator if walking activity detected (triggers step 2 refinement on next rest)
        if (result.stateCode == STATE_CODE_WALK) {
            imuCalibrator.notifyWalkDetected();
        }

        if (result.isFall) {
            Serial.printf("[EDGE-AI] *** CRITICAL FALL DETECTED: %s (Confidence: %.1f%%, Impact: %.1f m/s2, Latency: %u us) ***\n",
                          result.className, result.confidence * 100.0f, result.peakImpactMs2, result.inferenceTimeUs);

            // Trigger emergency haptic feedback alert
            haptic.play(HAPTIC_PATTERN_ALERT_PULSE, 255, 500);

            if (bleServer.isConnected()) {
                char diagBuf[64];
                snprintf(diagBuf, sizeof(diagBuf), "AI_FALL:%s %u%% pk=%.1f",
                         result.className, (unsigned int)(result.confidence * 100.0f), result.peakImpactMs2);
                bleServer.notifyStudioControl(diagBuf);

                // Notify connected mobile client over BLE with critical fall flags
                bleServer.notifyActivity(
                    result.stateCode,
                    (uint8_t)(result.confidence * 100.0f),
                    epochSec,
                    DETECTION_FLAG_CRITICAL_FALL | DETECTION_FLAG_LOCAL_HAPTIC
                );
            } else {
                // BLE disconnected: persist fall alert to non-volatile flash memory
                offlineFallStorage.saveFall(
                    result.stateCode,
                    (uint8_t)(result.confidence * 100.0f),
                    epochSec
                );
            }
            lastReportedActivityState = result.stateCode;
            if (bleServer.isHitlEnabled() && !hitlCapturePending) {
                hitlCapturePending = true;
                hitlPostSamplesRemaining = HITL_POST_TRIGGER_SAMPLES;
                hitlTriggerEpochSec = epochSec;
            }
        } else if (result.stateCode != lastReportedActivityState) {
            Serial.printf("[EDGE-AI] Activity State Changed: %s (Confidence: %.1f%%, Impact: %.1f m/s2, Latency: %u us)\n",
                          result.className, result.confidence * 100.0f, result.peakImpactMs2, result.inferenceTimeUs);

            if (bleServer.isConnected()) {
                char diagBuf[64];
                snprintf(diagBuf, sizeof(diagBuf), "AI:%s %u%% pk=%.1f",
                         result.className, (unsigned int)(result.confidence * 100.0f), result.peakImpactMs2);
                bleServer.notifyStudioControl(diagBuf);

                bleServer.notifyActivity(
                    result.stateCode,
                    (uint8_t)(result.confidence * 100.0f),
                    epochSec,
                    0
                );
            }
            lastReportedActivityState = result.stateCode;
            if (bleServer.isHitlEnabled() && !hitlCapturePending) {
                hitlCapturePending = true;
                hitlPostSamplesRemaining = HITL_POST_TRIGGER_SAMPLES;
                hitlTriggerEpochSec = epochSec;
            }
        }
    }
}

void SensorPipeline::checkIdleRestConfirmation(uint32_t nowMs) {
    if (bleServer.isConnected() && hasPendingIdleConfirmation && (nowMs - lastStepDetectedMs >= STEP_IDLE_CONFIRM_DELAY_MS)) {
        hasPendingIdleConfirmation = false;
        StepCounterPayload payload = stepDetector.getPayload(nowMs);
        bleServer.notifyStepCounter(payload);
        lastNotifiedTotalSteps = stepDetector.getTotalSteps();
        Serial.printf("[STEP] Idle rest confirmation sent: Total=%u | Cadence=%u SPM\n",
                      lastNotifiedTotalSteps, payload.cadence_spm);
    }
}
