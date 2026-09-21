#include <Arduino.h>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "ble/ble_server.h"
#include "studio/studio_manager.h"
#include "activity_detector.h"
#include "imu_calibrator.h"

// Hardware module and service instances
static ImuMpu6050 imu;
static HapticDriver haptic;
static HealthKicksBleServer bleServer;
static StudioManager studioManager;
static ActivityDetector activityDetector;
static ImuCalibrator imuCalibrator;

// Timers and scheduling
static uint32_t lastImuReadMs = 0;
static uint32_t lastDiagnosticPrintMs = 0;
static bool imuReady = false;
static uint8_t lastReportedActivityState = 0xFF;

/**
 * @brief Runs the hardware self-test diagnostic suite at startup.
 */
static void runHardwareDiagnostics() {
    Serial.println("\n========================================================");
    Serial.println("   HEALTHKICKS ESP32-S3-N16R8 - HARDWARE SELF-TEST      ");
    Serial.println("========================================================");

    // 1. Octal PSRAM & Flash memory diagnostics
    uint32_t psramSize = ESP.getPsramSize();
    uint32_t freePsram = ESP.getFreePsram();
    uint32_t heapSize = ESP.getHeapSize();
    uint32_t freeHeap = ESP.getFreeHeap();

    Serial.printf("[MEMORY] Total PSRAM : %u bytes (%.2f MB)\n", psramSize, psramSize / (1024.0 * 1024.0));
    Serial.printf("[MEMORY] Free PSRAM  : %u bytes (%.2f MB)\n", freePsram, freePsram / (1024.0 * 1024.0));
    Serial.printf("[MEMORY] Internal SRAM: %u bytes (Free: %u bytes)\n", heapSize, freeHeap);

    if (psramSize >= 7 * 1024 * 1024) {
        Serial.println("[PASS] 8MB Octal PSRAM detected and operational.");
    } else {
        Serial.println("[WARN] PSRAM less than 8MB or not initialized (Check platformio.ini opi_opi).");
    }

    // 2. I2C Bus & MPU-6050 IMU Sensor diagnostics
    Serial.printf("[IMU] Initializing I2C bus (SDA=%d, SCL=%d, Freq=%d Hz)...\n", PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_FREQ_HZ);
    imuReady = imu.begin(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_FREQ_HZ);
    if (imuReady) {
        uint8_t who = imu.readWhoAmI();
        Serial.printf("[PASS] MPU-6050 detected at address 0x%02X (WHO_AM_I = 0x%02X).\n", IMU_I2C_ADDR, who);

        float ax, ay, az, gx, gy, gz;
        if (imu.readRawMetrics(ax, ay, az, gx, gy, gz)) {
            Serial.printf("[DATA] Accel: [%.2fg, %.2fg, %.2fg] | Gyro: [%.1fdps, %.1fdps, %.1fdps]\n",
                          ax, ay, az, gx, gy, gz);
        }
    } else {
        Serial.printf("[FAIL] MPU-6050 not found at address 0x%02X. Check SDA/SCL wiring and 4.7k pull-ups.\n", IMU_I2C_ADDR);
    }

    // 3. Haptic Actuator diagnostics (PWM Vibrator on GPIO 7)
    Serial.printf("[HAPTIC] Initializing PWM on GPIO %d (Freq: %d Hz)...\n", PIN_HAPTIC_PWM, HAPTIC_LEDC_FREQ_HZ);
    haptic.init(PIN_HAPTIC_PWM);
    Serial.println("[PASS] Haptic driver initialized (Anti-glitch LOW, ready for commands).");

    Serial.println("========================================================\n");
}

void setup() {
    // Initialize native USB CDC serial monitor
    Serial.begin(115200);
    delay(1000); // Allow USB CDC port time to enumerate

    // Configure pairing button on GPIO 14
    pinMode(PIN_BTN_PAIRING, INPUT_PULLUP);

    // Configure Deep Sleep wakeup (ext1 on GPIO 14 active low)
    CONFIGURE_EXT1_WAKEUP();

    // Initial hardware diagnostics
    runHardwareDiagnostics();

    // Configure Studio capture manager
    studioManager.begin(&bleServer, &haptic, &imu);

    // Configure Edge AI activity detection engine
    activityDetector.begin(0.75f, 5000, 18.0f);
    Serial.printf("[EDGE-AI] Activity classifier ready (Window: %.1fs / %d samples, %d classes, Thresh: %.0f%%).\n",
                  MODEL_WINDOW_SIZE_SEC, MODEL_WINDOW_SAMPLES, MODEL_CLASS_COUNT, activityDetector.getConfidenceThreshold() * 100.0f);

    // Configure Dynamic IMU Tilt Calibrator (4.0s stillness window @ 19 Hz)
    imuCalibrator.begin(IMU_SAMPLE_FREQ_HZ, 4.0f);
    imuCalibrator.setOnCalibrationComplete([](bool success, int step) {
        if (success) {
            // Trigger confirmation haptic pulse strictly after calibration and NVS persistence
            haptic.play(HAPTIC_PATTERN_DOUBLE_PULSE, 200, 250);
            bleServer.notifyStudioControl("CALIBRATION_OK");
            Serial.printf("[CALIB] Calibration sequence completed (step %d), notified mobile client.\n", step);
        } else {
            bleServer.notifyStudioControl("CALIBRATION_ERROR motion_detected");
        }
    });

    // Configure BLE interoperability callbacks
    bleServer.setHapticCallback([](uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
        haptic.play(pattern, intensity, durationMs);
    });

    bleServer.setStudioCommandCallback([](const String& command) {
        studioManager.handleCommand(command.c_str());
    });

    bleServer.setCalibrationCallback([]() {
        bleServer.notifyStudioControl("CALIBRATING 4.0");
        imuCalibrator.triggerManualCalibration();
    });

    // Start NimBLE server
    Serial.printf("[BLE] Starting NimBLE server with name \"%s\"...\n", BLE_DEVICE_NAME);
    bleServer.begin(BLE_DEVICE_NAME);
}

void loop() {
    uint32_t now = millis();

    // 0. Asynchronous and reliable BLE advertising restart if disconnected (avoids radio deadlocks)
    if (g_need_restart_advertising) {
        g_need_restart_advertising = false;
        delay(50);
        NimBLEDevice::getAdvertising()->start();
        Serial.println("[BLE] Advertising restarted.");
    }

    // Clean cancellation of Studio session on unexpected disconnection
    if (!bleServer.isConnected() && studioManager.isBusy()) {
        studioManager.cancel();
    }

    // 1. Update haptic actuator and Studio manager
    haptic.update();
    studioManager.update();

    // 2. Nominal IMU acquisition & Edge AI Activity Detection (only if Studio is not recording)
    if (!studioManager.isRecording() && (now - lastImuReadMs >= IMU_SAMPLE_PERIOD_MS)) {
        lastImuReadMs = now;

        if (imuReady) {
            float ax, ay, az, gx, gy, gz;
            if (imu.readRawMetrics(ax, ay, az, gx, gy, gz)) {
                // Ingest raw physical readings into dynamic orientation calibrator
                imuCalibrator.update(ax, ay, az, gx, gy, gz);

                // Apply dynamic alignment rotation matrix (corrects sensor PCB tilt)
                imuCalibrator.applyCalibration(ax, ay, az, gx, gy, gz);

                // Push calibrated sample into sliding buffer
                activityDetector.pushSample(ax, ay, az, gx, gy, gz);

                // Run real-time edge inference if step interval is due
                if (activityDetector.isEvaluationDue()) {
                    ActivityDetectionResult result;
                    if (activityDetector.detect(result)) {
                        if (result.confidence >= activityDetector.getConfidenceThreshold()) {
                            uint32_t epochSec = (uint32_t)(now / 1000);

                            // Notify calibrator if walking activity detected (triggers step 2 refinement on next rest)
                            if (result.stateCode == STATE_CODE_WALK) {
                                imuCalibrator.notifyWalkDetected();
                            }

                            if (result.isFall) {
                                Serial.printf("[EDGE-AI] *** CRITICAL FALL DETECTED: %s (Confidence: %.1f%%, Latency: %u us) ***\n",
                                              result.className, result.confidence * 100.0f, result.inferenceTimeUs);

                                // Trigger emergency haptic feedback alert
                                haptic.play(HAPTIC_PATTERN_ALERT_PULSE, 255, 500);

                                // Notify connected mobile client over BLE with critical fall flags
                                bleServer.notifyActivity(
                                    result.stateCode,
                                    (uint8_t)(result.confidence * 100.0f),
                                    epochSec,
                                    DETECTION_FLAG_CRITICAL_FALL | DETECTION_FLAG_LOCAL_HAPTIC
                                );
                                lastReportedActivityState = result.stateCode;
                            } else if (result.stateCode != lastReportedActivityState) {
                                Serial.printf("[EDGE-AI] Activity State Changed: %s (Confidence: %.1f%%, Latency: %u us)\n",
                                              result.className, result.confidence * 100.0f, result.inferenceTimeUs);

                                bleServer.notifyActivity(
                                    result.stateCode,
                                    (uint8_t)(result.confidence * 100.0f),
                                    epochSec,
                                    0
                                );
                                lastReportedActivityState = result.stateCode;
                            }
                        }
                    }
                }
            }
        }
    }

    // 3. Periodic debug telemetry output (every 3 seconds)
    if (now - lastDiagnosticPrintMs >= 3000) {
        lastDiagnosticPrintMs = now;

        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        if (imuReady) {
            imu.readRawMetrics(ax, ay, az, gx, gy, gz);
        }

        Serial.printf("[HEARTBEAT] BLE: %s | MTU: %d | IMU: [%.2f, %.2f, %.2f] g | Free PSRAM: %u KB\n",
                      bleServer.isConnected() ? "CONNECTED" : "ADVERTISING",
                      bleServer.getNegotiatedMtu(),
                      ax, ay, az,
                      ESP.getFreePsram() / 1024);
    }

    // 4. Handle inactivity and switch to Deep Sleep after 3 minutes without connection
    if (!bleServer.isConnected()) {
        uint32_t inactiveMs = now - bleServer.getLastActivityTime();
        if (inactiveMs >= DEEP_SLEEP_TIMEOUT_MS) {
            Serial.printf("[POWER] Inactivity of %u seconds reached. Entering Deep Sleep...\n", BLE_ADVERTISING_TIMEOUT_SEC);
            Serial.println("[POWER] Press GPIO 14 button to wake up the shoe.");
            Serial.flush();

            bleServer.stopAdvertising();
            haptic.stop();

            // Enter deep sleep
            esp_deep_sleep_start();
        }
    }

    // 5. Handle power/pairing switch (GPIO 14 - Active LOW with internal pull-up)
    static bool s_switchState = HIGH;
    static uint32_t s_lastSwitchCheckMs = 0;

    if (now - s_lastSwitchCheckMs >= 50) { // Sample every 50 ms
        s_lastSwitchCheckMs = now;
        bool reading = digitalRead(PIN_BTN_PAIRING);

        if (reading != s_switchState) {
            s_switchState = reading;
            if (s_switchState == LOW) {
                Serial.println("[SWITCH] Position ON (grounded): restarting BLE advertising");
                haptic.play(HAPTIC_PATTERN_CONTINUOUS, 180, 80);
                if (!bleServer.isConnected()) {
                    NimBLEDevice::getAdvertising()->start();
                }
            } else {
                Serial.println("[SWITCH] Position OFF (open)");
                haptic.play(HAPTIC_PATTERN_CONTINUOUS, 120, 50);
            }
        }
    }
}

