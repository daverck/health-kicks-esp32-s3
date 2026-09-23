#include "power_manager.h"
#include "sensors/imu_mpu6050.h"
#include "ble/ble_server.h"
#include "actuators/haptic_driver.h"
#include "step_detector.h"
#include "imu_calibrator.h"
#include <NimBLEDevice.h>

// RTC Fast Memory variables preserved across Deep Sleep resets
RTC_DATA_ATTR static uint32_t rtc_total_steps = 0;
RTC_DATA_ATTR static uint16_t rtc_walk_steps = 0;
RTC_DATA_ATTR static uint16_t rtc_run_steps = 0;
RTC_DATA_ATTR static uint16_t rtc_stairs_steps = 0;
RTC_DATA_ATTR static uint16_t rtc_unclassified_steps = 0;
RTC_DATA_ATTR static uint32_t rtc_boot_count = 0;
RTC_DATA_ATTR static bool rtc_is_calibrated = false;
RTC_DATA_ATTR static float rtc_calibration_matrix[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

// Static member definitions
uint32_t PowerManager::_lastActivityMs = 0;
uint32_t PowerManager::_inactivityTimeoutSec = DEEP_SLEEP_INACTIVITY_TIMEOUT_SEC;
esp_sleep_wakeup_cause_t PowerManager::_wakeupCause = ESP_SLEEP_WAKEUP_UNDEFINED;

void PowerManager::init(StepDetector* stepDetector, ImuCalibrator* imuCalibrator) {
    rtc_boot_count++;
    _wakeupCause = esp_sleep_get_wakeup_cause();
    _lastActivityMs = millis();

    switch (_wakeupCause) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.printf("[POWER] Woke up from Deep Sleep by Motion (GPIO %d) | Boot count: %u\n",
                          PIN_IMU_INT, rtc_boot_count);
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            Serial.printf("[POWER] Woke up from Deep Sleep by Manual Switch (GPIO %d) | Boot count: %u\n",
                          PIN_BTN_PAIRING, rtc_boot_count);
            break;
        default:
            Serial.printf("[POWER] Cold boot / Power-on reset | Boot count: %u\n", rtc_boot_count);
            break;
    }

    // If waking up from sleep, restore persisted counters and orientation
    if (_wakeupCause == ESP_SLEEP_WAKEUP_EXT0 || _wakeupCause == ESP_SLEEP_WAKEUP_EXT1) {
        if (stepDetector != nullptr) {
            stepDetector->restoreState(rtc_total_steps, rtc_walk_steps, rtc_run_steps,
                                      rtc_stairs_steps, rtc_unclassified_steps);
            Serial.printf("[POWER] Restored step counters: Total=%u (W:%u, R:%u, S:%u, U:%u)\n",
                          rtc_total_steps, rtc_walk_steps, rtc_run_steps,
                          rtc_stairs_steps, rtc_unclassified_steps);
        }

        if (imuCalibrator != nullptr && rtc_is_calibrated) {
            imuCalibrator->restoreFromMatrix(rtc_calibration_matrix);
            Serial.println("[POWER] Restored IMU alignment matrix from RTC fast memory.");
        }
    }
}

void PowerManager::recordActivity(uint32_t nowMs) {
    _lastActivityMs = nowMs;
}

void PowerManager::checkSleepConditions(bool isBleConnected,
                                       bool isRecording,
                                       uint32_t nowMs,
                                       ImuMpu6050& imu,
                                       HealthKicksBleServer& bleServer,
                                       HapticDriver& haptic,
                                       StepDetector& stepDetector,
                                       ImuCalibrator& imuCalibrator) {
    // Keep alive if a client is connected or a Studio capture is recording
    if (isBleConnected || isRecording) {
        _lastActivityMs = nowMs;
        return;
    }

    // Check inactivity timeout
    uint32_t elapsedMs = nowMs - _lastActivityMs;
    if (elapsedMs >= (_inactivityTimeoutSec * 1000UL)) {
        enterDeepSleep(imu, bleServer, haptic, stepDetector, imuCalibrator);
    }
}

void PowerManager::enterDeepSleep(ImuMpu6050& imu,
                                 HealthKicksBleServer& bleServer,
                                 HapticDriver& haptic,
                                 StepDetector& stepDetector,
                                 ImuCalibrator& imuCalibrator) {
    Serial.printf("\n[POWER] Inactivity timeout (%u s) reached without connection. Entering Deep Sleep (~10 uA)...\n",
                  _inactivityTimeoutSec);

    // 1. Snapshot state to RTC Fast Memory
    rtc_total_steps = stepDetector.getTotalSteps();
    rtc_walk_steps = stepDetector.getWalkSteps();
    rtc_run_steps = stepDetector.getRunSteps();
    rtc_stairs_steps = stepDetector.getStairsSteps();
    rtc_unclassified_steps = stepDetector.getUnclassifiedSteps();
    rtc_is_calibrated = imuCalibrator.isCalibrated();
    imuCalibrator.getRotationMatrixFlat(rtc_calibration_matrix);

    Serial.printf("[POWER] State saved to RTC: Total Steps=%u (W:%u, R:%u, S:%u, U:%u), Calibrated=%d\n",
                  rtc_total_steps, rtc_walk_steps, rtc_run_steps,
                  rtc_stairs_steps, rtc_unclassified_steps, rtc_is_calibrated);

    // 2. Stop haptic motor and BLE stack cleanly
    haptic.stop();
    bleServer.stopAdvertising();
    NimBLEDevice::deinit(true);
    Serial.println("[POWER] BLE stack stopped and de-initialized.");

    // 3. Put MPU-6050 in ultra-low power Wake-On-Motion mode
    imu.enableWakeOnMotion(IMU_WOM_THRESHOLD, IMU_WOM_DURATION);
    Serial.printf("[POWER] MPU-6050 WOM mode configured on GPIO %d (Thresh=%u, Dur=%u).\n",
                  PIN_IMU_INT, IMU_WOM_THRESHOLD, IMU_WOM_DURATION);

    // 4. Arm wakeup sources: EXT0 (MPU-6050 INT GPIO 6 HIGH) and EXT1 (GPIO 14 LOW)
    esp_sleep_enable_ext0_wakeup(PIN_IMU_INT, 1);
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_BTN_PAIRING, ESP_EXT1_WAKEUP_ANY_LOW);
    Serial.println("[POWER] Wakeup sources armed: EXT0 (GPIO 6 HIGH - Motion) & EXT1 (GPIO 14 LOW - Button).");
    Serial.println("[POWER] Entering Deep Sleep now. Goodbye!\n");
    Serial.flush();

    // 5. Enter Deep Sleep
    esp_deep_sleep_start();
}

uint32_t PowerManager::getBootCount() {
    return rtc_boot_count;
}

esp_sleep_wakeup_cause_t PowerManager::getWakeupCause() {
    return _wakeupCause;
}

void PowerManager::setTimeoutSec(uint32_t sec) {
    _inactivityTimeoutSec = sec > 0 ? sec : DEEP_SLEEP_INACTIVITY_TIMEOUT_SEC;
}

uint32_t PowerManager::getTimeoutSec() {
    return _inactivityTimeoutSec;
}
