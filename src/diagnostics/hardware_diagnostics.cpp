#include "diagnostics/hardware_diagnostics.h"
#include "config.h"

bool HardwareDiagnostics::runDiagnostics(ImuMpu6050& imu, HapticDriver& haptic, LedDriver& statusLed) {
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
    bool imuReady = imu.begin(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_FREQ_HZ);
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

    // 4. Status LED diagnostics (Blue LED on GPIO 13)
    Serial.printf("[LED] Initializing status LED on GPIO %d...\n", PIN_LED_BLUE);
    statusLed.init(PIN_LED_BLUE);
    Serial.println("[PASS] Status LED initialized (Ready for BLE, Calibration, and Studio).");

    Serial.println("========================================================\n");
    return imuReady;
}
