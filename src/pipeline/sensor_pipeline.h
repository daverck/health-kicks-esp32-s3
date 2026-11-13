#pragma once

#include <Arduino.h>
#include <vector>
#include <lwip/def.h>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "imu_calibrator.h"
#include "step_detector.h"
#include "inactivity_monitor.h"
#include "activity_detector.h"
#include "ble/ble_server.h"
#include "actuators/haptic_driver.h"
#include "studio/studio_manager.h"
#include "offline_fall_storage.h"

/// Modular acquisition, calibration, step detection, HITL buffering, and Edge AI evaluation pipeline.
class SensorPipeline {
public:
    static const size_t HITL_BUFFER_CAPACITY = 76;       // 4.0s @ 19 Hz = 76 samples
    static const size_t HITL_POST_TRIGGER_SAMPLES = 28;  // ~1.5s post-event (28 * 53ms ≈ 1.48s)
    static const uint32_t STEP_NOTIFY_BATCH_COUNT = 50;
    static const uint32_t STEP_IDLE_CONFIRM_DELAY_MS = 3000;

    SensorPipeline(
        ImuMpu6050& imu,
        ImuCalibrator& imuCalibrator,
        StepDetector& stepDetector,
        InactivityMonitor& inactivityMonitor,
        ActivityDetector& activityDetector,
        HealthKicksBleServer& bleServer,
        HapticDriver& haptic,
        StudioManager& studioManager,
        OfflineFallStorage& offlineFallStorage
    );

    /// Processes IMU sampling, filtering, step detection, and Edge AI inference if due.
    void process(uint32_t nowMs, bool imuReady);

    /// Handles initial step sync when BLE connects.
    void onBleConnected(uint32_t nowMs);

    /// Returns the latest activity state detected.
    uint8_t getCurrentActivityState() const { return currentActivityState; }

    /// Returns the most recent raw and calibrated accelerometer readings (in g) captured by the 19 Hz pipeline.
    void getLastAccelSample(
        float& rawAx, float& rawAy, float& rawAz,
        float& calibAx, float& calibAy, float& calibAz
    ) const {
        rawAx = lastRawAx;
        rawAy = lastRawAy;
        rawAz = lastRawAz;
        calibAx = lastCalibAx;
        calibAy = lastCalibAy;
        calibAz = lastCalibAz;
    }

private:
    ImuMpu6050& imu;
    ImuCalibrator& imuCalibrator;
    StepDetector& stepDetector;
    InactivityMonitor& inactivityMonitor;
    ActivityDetector& activityDetector;
    HealthKicksBleServer& bleServer;
    HapticDriver& haptic;
    StudioManager& studioManager;
    OfflineFallStorage& offlineFallStorage;

    uint32_t lastImuReadMs = 0;
    uint8_t lastReportedActivityState = 0xFF;
    uint8_t currentActivityState = STATE_CODE_IDLE;

    // Latest cached accelerometer sample for zero-jitter diagnostic telemetry
    float lastRawAx = 0.0f;
    float lastRawAy = 0.0f;
    float lastRawAz = 0.0f;
    float lastCalibAx = 0.0f;
    float lastCalibAy = 0.0f;
    float lastCalibAz = 0.0f;

    // Step notification batching
    uint32_t lastNotifiedTotalSteps = 0;
    uint32_t lastStepDetectedMs = 0;
    bool hasPendingIdleConfirmation = false;

    // Human-in-the-Loop (HITL) rolling frame buffer
    ImuRawFrame hitlRingBuffer[HITL_BUFFER_CAPACITY];
    size_t hitlRingHead = 0;
    size_t hitlBufferCount = 0;
    bool hitlCapturePending = false;
    size_t hitlPostSamplesRemaining = 0;
    uint32_t hitlTriggerEpochSec = 0;

    void processHitlBuffer(float ax, float ay, float az, float gx, float gy, float gz, uint32_t nowMs);
    void evaluateEdgeAi(uint32_t nowMs);
    void checkIdleRestConfirmation(uint32_t nowMs);
};
