#include <Arduino.h>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "ble/ble_server.h"
#include "studio/studio_manager.h"

// Hardware module and service instances
static ImuMpu6050 imu;
static HapticDriver haptic;
static HealthKicksBleServer bleServer;
static StudioManager studioManager;

// Timers and scheduling
static uint32_t lastImuReadMs = 0;
static uint32_t lastDiagnosticPrintMs = 0;
static bool imuReady = false;

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

    // Configure BLE interoperability callbacks
    bleServer.setHapticCallback([](uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
        haptic.play(pattern, intensity, durationMs);
    });

    bleServer.setStudioCommandCallback([](const String& command) {
        studioManager.handleCommand(command.c_str());
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

    // 2. Nominal IMU acquisition (only if Studio is not recording)
    if (!studioManager.isRecording() && (now - lastImuReadMs >= IMU_SAMPLE_PERIOD_MS)) {
        lastImuReadMs = now;

        if (imuReady) {
            ImuRawFrame frame;
            if (imu.readFrame(frame, (uint16_t)(now & 0xFFFF))) {
                // In nominal connected mode
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

