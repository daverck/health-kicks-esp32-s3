#include <Arduino.h>
#include <algorithm>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "actuators/led_driver.h"
#include "ble/ble_server.h"
#include "studio/studio_manager.h"
#include "activity_detector.h"
#include "imu_calibrator.h"
#include "step_detector.h"
#include "inactivity_monitor.h"
#include "power_manager.h"
#include "offline_fall_storage.h"
#include "diagnostics/hardware_diagnostics.h"
#include "pipeline/sensor_pipeline.h"
#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <lwip/def.h>

// Hardware module and service instances
static ImuMpu6050 imu;
static HapticDriver haptic;
static LedDriver statusLed;
static HealthKicksBleServer bleServer;
static StudioManager studioManager;
static ActivityDetector activityDetector;
static ImuCalibrator imuCalibrator;
static StepDetector stepDetector;
static InactivityMonitor inactivityMonitor;
static OfflineFallStorage offlineFallStorage;
static SensorPipeline sensorPipeline(
    imu,
    imuCalibrator,
    stepDetector,
    inactivityMonitor,
    activityDetector,
    bleServer,
    haptic,
    studioManager,
    offlineFallStorage
);
static Preferences configPrefs;
static const char* CONFIG_NVS_NAMESPACE = "hk_config";

// Timers and scheduling
static uint32_t lastDiagnosticPrintMs = 0;
static bool imuReady = false;
static bool prevBleConnected = false;

// Anti-Bricking & Rollback Probationary State
static bool otaPendingVerification = false;
static uint32_t otaVerificationStartMs = 0;
static const uint32_t OTA_VERIFY_PROBATION_PERIOD_MS = 15000; // 15 seconds probationary window

void setup() {
    // Initialize native USB CDC serial monitor
    Serial.begin(115200);
    delay(1000); // Allow USB CDC port time to enumerate

    // Configure pairing button on GPIO 14
    pinMode(PIN_BTN_PAIRING, INPUT_PULLUP);

    // Configure Blue Status LED on GPIO 13
    statusLed.init(PIN_LED_BLUE);
    statusLed.setBleAdvertising();

    // Initial hardware diagnostics
    imuReady = HardwareDiagnostics::runDiagnostics(imu, haptic, statusLed);

    // Check if running in probationary state (Anti-Bricking & Rollback)
    const esp_partition_t* runningPartition = esp_ota_get_running_partition();
    esp_ota_img_states_t otaState;
    if (runningPartition && esp_ota_get_state_partition(runningPartition, &otaState) == ESP_OK) {
        if (otaState == ESP_OTA_IMG_PENDING_VERIFY) {
            otaPendingVerification = true;
            otaVerificationStartMs = millis();
            Serial.println("\n[OTA-ROLLBACK] ========================================================");
            Serial.println("[OTA-ROLLBACK] *** FIRMWARE RUNNING IN PROBATIONARY STATE (PENDING_VERIFY) ***");
            Serial.printf("[OTA-ROLLBACK] Active slot: %s (address: 0x%08X, size: %u bytes)\n",
                          runningPartition->label, runningPartition->address, runningPartition->size);

            if (!imuReady) {
                Serial.println("[OTA-ROLLBACK] CRITICAL FAILURE: IMU sensor failed self-test on new firmware!");
                Serial.println("[OTA-ROLLBACK] Triggering immediate automatic rollback to previous slot...");
                delay(1000);
                esp_ota_mark_app_invalid_rollback_and_reboot();
            } else {
                Serial.println("[OTA-ROLLBACK] Hardware self-test PASSED. Starting 15s probation timer...");
            }
            Serial.println("[OTA-ROLLBACK] ========================================================\n");
        } else if (otaState == ESP_OTA_IMG_VALID) {
            Serial.printf("[OTA-ROLLBACK] Running partition '%s' is confirmed VALID.\n", runningPartition->label);
        }
    }

    // Configure Studio capture manager with dynamic tilt calibrator
    studioManager.begin(&bleServer, &haptic, &imu, &imuCalibrator);

    // Configure Edge AI activity detection engine
    configPrefs.begin(CONFIG_NVS_NAMESPACE, false);
    
    // Load fall confidence threshold from NVS or use default
    float fallConfidenceThreshold = DEFAULT_FALL_CONFIDENCE_THRESHOLD;
    if (configPrefs.isKey("fall_confidence_threshold")) {
        fallConfidenceThreshold = configPrefs.getFloat("fall_confidence_threshold", DEFAULT_FALL_CONFIDENCE_THRESHOLD);
        // Clamp to valid range
        fallConfidenceThreshold = std::max(MIN_FALL_CONFIDENCE_THRESHOLD, 
                                          std::min(MAX_FALL_CONFIDENCE_THRESHOLD, fallConfidenceThreshold));
    } else {
        // Save default value if not present
        configPrefs.putFloat("fall_confidence_threshold", fallConfidenceThreshold);
    }

    // Load fall cooldown from NVS or use default
    uint32_t fallCooldownMs = DEFAULT_FALL_COOLDOWN_MS;
    if (configPrefs.isKey("fall_cooldown_ms")) {
        fallCooldownMs = configPrefs.getUInt("fall_cooldown_ms", DEFAULT_FALL_COOLDOWN_MS);
        fallCooldownMs = std::max((uint32_t)MIN_FALL_COOLDOWN_MS,
                                  std::min((uint32_t)MAX_FALL_COOLDOWN_MS, fallCooldownMs));
    } else {
        configPrefs.putUInt("fall_cooldown_ms", fallCooldownMs);
    }
    
    activityDetector.begin(0.50f, fallCooldownMs, fallConfidenceThreshold);
    Serial.printf("[EDGE-AI] Activity classifier ready (Window: %.1fs / %d samples, %d classes, Thresh: %.0f%%, Fall Conf: %.2f, Fall Cooldown: %u ms).\n",
                  MODEL_WINDOW_SIZE_SEC, MODEL_WINDOW_SAMPLES, MODEL_CLASS_COUNT,
                  activityDetector.getConfidenceThreshold() * 100.0f, fallConfidenceThreshold, fallCooldownMs);

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

    // Configure Deterministic Step Detector
    stepDetector.begin();

    // Initialize Power Manager and restore persisted state from RTC memory
    PowerManager::init(&stepDetector, &imuCalibrator);

    // Check if woken up by manual button press on GPIO 14
    if (PowerManager::getWakeupCause() == ESP_SLEEP_WAKEUP_EXT1) {
        haptic.play(HAPTIC_PATTERN_CONTINUOUS, 150, 100);
        Serial.println("[POWER] Woken up by Manual Button (GPIO 14). Welcome back!");
    }

    // Configure Autonomous Inactivity Monitor
    inactivityMonitor.begin();
    inactivityMonitor.setInactivityAlertCallback([](uint32_t nowMs) {
        haptic.play(HAPTIC_PATTERN_DOUBLE_PULSE, 160, 200);
        uint32_t epochSec = (uint32_t)(nowMs / 1000);
        bleServer.notifyActivity(STATE_CODE_INACTIVITY_ALERT, 100, epochSec, DETECTION_FLAG_LOCAL_HAPTIC);
        PowerManager::recordActivity(nowMs);
        Serial.printf("[INACTIVITY] Emitted 0x20 alert notification & discrete haptic pulse at %u ms\n", nowMs);
    });

    // Configure BLE interoperability callbacks
    bleServer.setHapticCallback([](uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
        PowerManager::recordActivity(millis());
        haptic.play(pattern, intensity, durationMs);
    });

    bleServer.setStudioCommandCallback([](const std::string& command) {
        PowerManager::recordActivity(millis());
        studioManager.handleCommand(command);
    });

    bleServer.setCalibrationCallback([]() {
        PowerManager::recordActivity(millis());
        bleServer.notifyStudioControl("CALIBRATING 4.0");
        imuCalibrator.triggerManualCalibration();
    });

    bleServer.setInactivityConfigCallback([](bool enabled, uint16_t threshSec, uint16_t coolSec) {
        PowerManager::recordActivity(millis());
        inactivityMonitor.configure(enabled, threshSec, coolSec);
    });

    bleServer.setFallConfidenceConfigCallback([](float threshold) {
        PowerManager::recordActivity(millis());
        // Validate and clamp threshold
        threshold = std::max(MIN_FALL_CONFIDENCE_THRESHOLD, 
                            std::min(MAX_FALL_CONFIDENCE_THRESHOLD, threshold));
        activityDetector.setFallConfidenceThreshold(threshold);
        configPrefs.putFloat("fall_confidence_threshold", threshold);
        Serial.printf("[BLE] Fall confidence threshold updated to %.2f and persisted to NVS.\n", threshold);
    });

    bleServer.setFallCooldownConfigCallback([](uint32_t cooldownMs) {
        PowerManager::recordActivity(millis());
        cooldownMs = std::max((uint32_t)MIN_FALL_COOLDOWN_MS,
                              std::min((uint32_t)MAX_FALL_COOLDOWN_MS, cooldownMs));
        activityDetector.setFallCooldown(cooldownMs);
        configPrefs.putUInt("fall_cooldown_ms", cooldownMs);
        Serial.printf("[BLE] Fall cooldown updated to %u ms and persisted to NVS.\n", cooldownMs);
    });

    bleServer.setOtaStateChangeCallback([](bool active) {
        statusLed.setOtaUpdating(active);
        if (active) {
            Serial.println("[MAIN] OTA Update started: IMU sampling & Edge AI paused.");
        } else {
            Serial.println("[MAIN] OTA Update ended or aborted. Resuming nominal operation.");
        }
    });

    // Start NimBLE server
    Serial.printf("[BLE] Starting NimBLE server with name \"%s\"...\n", BLE_DEVICE_NAME);
    bleServer.begin(BLE_DEVICE_NAME);

    // Initialize resilient offline fall storage in NVS flash
    offlineFallStorage.begin();

    // Open a 60-second pairing window automatically right after an OTA update or when no bonds exist in NVS
    if (otaPendingVerification || NimBLEDevice::getNumBonds() == 0) {
        bleServer.openPairingWindow(60000);
    }

    // Initialize Task Watchdog Timer (TWDT) with 5s timeout to protect against I2C/FreeRTOS deadlocks
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&twdt_config);
#else
    // ESP-IDF v4.x / Arduino ESP32 v2.x (timeout in seconds, panic on timeout)
    esp_task_wdt_init(5, true);
#endif
    esp_task_wdt_add(NULL);
    Serial.println("[SYSTEM] FreeRTOS Task Watchdog Timer initialized (5.0s timeout).");
}

void loop() {
    esp_task_wdt_reset();
    uint32_t now = millis();

    // Fast-path handling during OTA firmware flash: dedicate 100% CPU and Flash bandwidth to BLE stream
    if (bleServer.getOtaService().isOtaInProgress()) {
        if (!statusLed.isOtaUpdating()) {
            statusLed.setOtaUpdating(true);
        }
        statusLed.update(now);
        delay(2);
        return;
    } else if (statusLed.isOtaUpdating()) {
        statusLed.setOtaUpdating(false);
    }

    // Anti-Bricking probation monitoring: confirm stability after 15 seconds of nominal execution
    if (otaPendingVerification) {
        if (!imuReady) {
            Serial.println("[OTA-ROLLBACK] CRITICAL: IMU became unresponsive during probation period!");
            Serial.println("[OTA-ROLLBACK] Triggering emergency rollback to previous operational slot...");
            delay(1000);
            esp_ota_mark_app_invalid_rollback_and_reboot();
        } else if (now - otaVerificationStartMs >= OTA_VERIFY_PROBATION_PERIOD_MS) {
            otaPendingVerification = false;
            esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
            if (err == ESP_OK) {
                Serial.println("\n[OTA-ROLLBACK] ========================================================");
                Serial.println("[OTA-ROLLBACK] *** SUCCESS: Probation period passed without error! ***");
                Serial.println("[OTA-ROLLBACK] *** Firmware marked as VALID. Rollback cancelled.      ***");
                Serial.println("[OTA-ROLLBACK] ========================================================\n");
                haptic.play(HAPTIC_PATTERN_DOUBLE_PULSE, 180, 200);
            } else {
                Serial.printf("[OTA-ROLLBACK] Warning: esp_ota_mark_app_valid_cancel_rollback failed: %s (0x%X)\n",
                              esp_err_to_name(err), err);
            }
        }
    }

    bool bleConnected = bleServer.isConnected();
    if (bleConnected && !prevBleConnected) {
        PowerManager::onBleConnected();
        // Just connected: notify immediately to sync initial step state and signal LED connection confirmation
        statusLed.setBleConnected();
        sensorPipeline.onBleConnected(now);

        // Flush any offline falls buffered in NVS flash
        if (offlineFallStorage.hasPendingFalls()) {
            offlineFallStorage.flushToBle(bleServer);
        }
    } else if (!bleConnected && prevBleConnected) {
        // Just disconnected: signal alert pattern then return to advertising
        statusLed.setBleDisconnected();
        Serial.println("[BLE] Client disconnected, status LED set to disconnected alert.");
    }
    prevBleConnected = bleConnected;

    // 0. Asynchronous and reliable BLE advertising restart if disconnected (avoids radio deadlocks)
    if (g_need_restart_advertising) {
        g_need_restart_advertising = false;
        delay(50);
        NimBLEDevice::getAdvertising()->start();
        statusLed.setBleAdvertising();
        Serial.println("[BLE] Advertising restarted.");
    }

    // Clean cancellation of Studio session on unexpected disconnection
    if (!bleServer.isConnected() && studioManager.isBusy()) {
        studioManager.cancel();
    }

    // 1. Update haptic actuator, Studio manager, and status LED
    haptic.update();
    studioManager.update();
    statusLed.setCalibrating(imuCalibrator.isCalibrating());
    statusLed.setStudioRecording(studioManager.isRecording());
    statusLed.update(now);

    // 2. Nominal IMU acquisition, filtering, step detection & Edge AI Activity Detection
    sensorPipeline.process(now, imuReady);

    // 3. Periodic debug telemetry output (every 3 seconds) using cached 19 Hz pipeline sample
    if (now - lastDiagnosticPrintMs >= 3000) {
        lastDiagnosticPrintMs = now;

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        float c_ax = 0.0f, c_ay = 0.0f, c_az = 0.0f;
        sensorPipeline.getLastAccelSample(ax, ay, az, c_ax, c_ay, c_az);

        Serial.printf("[HEARTBEAT] BLE: %s | MTU: %d | Raw: [%.2f, %.2f, %.2f]g | Calib: [%.2f, %.2f, %.2f]g (%s) | Free PSRAM: %u KB\n",
                      bleServer.isConnected() ? "CONNECTED" : "ADVERTISING",
                      bleServer.getNegotiatedMtu(),
                      ax, ay, az,
                      c_ax, c_ay, c_az,
                      imuCalibrator.isCalibrated() ? "CALIBRATED" : "UNCALIBRATED",
                      ESP.getFreePsram() / 1024);
    }

    // 4. Inactivity evaluation and automatic Deep Sleep management (~10 uA)
    if (!otaPendingVerification) {
        PowerManager::checkSleepConditions(
            bleServer.isConnected(),
            studioManager.isRecording(),
            now,
            imu,
            bleServer,
            haptic,
            stepDetector,
            imuCalibrator
        );
    }

    // 5. Handle power/pairing button (GPIO 14 - Active LOW with internal pull-up)
    static const uint32_t BUTTON_LONG_PRESS_MS = 2500;
    static bool s_lastButtonReading = HIGH;
    static uint32_t s_pressStartMs = 0;
    static bool s_isPressed = false;
    static bool s_isShuttingDown = false;
    static uint32_t s_lastButtonCheckMs = 0;

    if (now - s_lastButtonCheckMs >= 20) { // Sample every 20 ms
        s_lastButtonCheckMs = now;
        bool reading = digitalRead(PIN_BTN_PAIRING);

        // Transition: HIGH -> LOW (Button Pressed)
        if (reading == LOW && s_lastButtonReading == HIGH) {
            s_pressStartMs = now;
            s_isPressed = true;
            PowerManager::recordActivity(now);
        }

        // While button is held LOW
        if (reading == LOW && s_isPressed && !s_isShuttingDown) {
            if (now - s_pressStartMs >= BUTTON_LONG_PRESS_MS) {
                s_isShuttingDown = true;
                Serial.println("[POWER] Long press detected (>= 2.5s). Shutting down device (Manual Power OFF)...");
                haptic.play(HAPTIC_PATTERN_ALERT_PULSE, 200, 300);
                digitalWrite(PIN_LED_BLUE, LOW);

                // Wait for user to release the button so release does not re-trigger wakeup
                while (digitalRead(PIN_BTN_PAIRING) == LOW) {
                    delay(20);
                }
                delay(150); // Debounce post-release

                PowerManager::enterDeepSleep(imu, bleServer, haptic, stepDetector, imuCalibrator, PowerManager::SLEEP_REASON_MANUAL_POWER_OFF);
            }
        }

        // Transition: LOW -> HIGH (Button Released)
        if (reading == HIGH && s_lastButtonReading == LOW) {
            if (s_isPressed && !s_isShuttingDown) {
                uint32_t pressDuration = now - s_pressStartMs;
                if (pressDuration < BUTTON_LONG_PRESS_MS && pressDuration >= 50) {
                    Serial.printf("[BUTTON] Short press (%u ms): opening BLE pairing window & restarting advertising\n", pressDuration);
                    haptic.play(HAPTIC_PATTERN_CONTINUOUS, 180, 80);
                    bleServer.openPairingWindow(60000);
                    if (!bleServer.isConnected()) {
                        NimBLEDevice::getAdvertising()->start();
                        statusLed.setBleAdvertising();
                    }
                }
            }
            s_isPressed = false;
            s_isShuttingDown = false;
        }

        s_lastButtonReading = reading;
    }
}
