#include <Arduino.h>
#include "config.h"
#include "invariants.h"
#include "sensors/imu_mpu6050.h"
#include "actuators/haptic_driver.h"
#include "ble/ble_server.h"

// Instances des modules matériels et services
static ImuMpu6050 imu;
static HapticDriver haptic;
static HealthKicksBleServer bleServer;

// Timers et cadencement
static uint32_t lastImuReadMs = 0;
static uint32_t lastDiagnosticPrintMs = 0;
static bool imuReady = false;

/**
 * @brief Exécute la suite de diagnostics matériels au démarrage.
 */
static void runHardwareDiagnostics() {
    Serial.println("\n========================================================");
    Serial.println("   HEALTHKICKS ESP32-S3-N16R8 - HARDWARE SELF-TEST      ");
    Serial.println("========================================================");

    // 1. Diagnostic de la mémoire Octal PSRAM & Flash
    uint32_t psramSize = ESP.getPsramSize();
    uint32_t freePsram = ESP.getFreePsram();
    uint32_t heapSize = ESP.getHeapSize();
    uint32_t freeHeap = ESP.getFreeHeap();

    Serial.printf("[MEMOIRE] PSRAM Totale : %u octets (%.2f MB)\n", psramSize, psramSize / (1024.0 * 1024.0));
    Serial.printf("[MEMOIRE] PSRAM Libre  : %u octets (%.2f MB)\n", freePsram, freePsram / (1024.0 * 1024.0));
    Serial.printf("[MEMOIRE] SRAM Interne : %u octets (Libre: %u octets)\n", heapSize, freeHeap);

    if (psramSize >= 7 * 1024 * 1024) {
        Serial.println("[PASS] Octal PSRAM 8MB détectée et opérationnelle.");
    } else {
        Serial.println("[WARN] PSRAM inférieure à 8MB ou non initialisée (Vérifier platformio.ini opi_opi).");
    }

    // 2. Diagnostic Bus I2C & Capteur IMU MPU-6050
    Serial.printf("[IMU] Initialisation bus I2C (SDA=%d, SCL=%d, Freq=%d Hz)...\n", PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_FREQ_HZ);
    imuReady = imu.begin(PIN_IMU_SDA, PIN_IMU_SCL, IMU_I2C_FREQ_HZ);
    if (imuReady) {
        uint8_t who = imu.readWhoAmI();
        Serial.printf("[PASS] MPU-6050 détecté à l'adresse 0x%02X (WHO_AM_I = 0x%02X).\n", IMU_I2C_ADDR, who);

        float ax, ay, az, gx, gy, gz;
        if (imu.readRawMetrics(ax, ay, az, gx, gy, gz)) {
            Serial.printf("[DATA] Accel: [%.2fg, %.2fg, %.2fg] | Gyro: [%.1fdps, %.1fdps, %.1fdps]\n",
                          ax, ay, az, gx, gy, gz);
        }
    } else {
        Serial.printf("[FAIL] MPU-6050 introuvable à l'adresse 0x%02X. Vérifier câblage SDA/SCL et pull-ups 4.7k.\n", IMU_I2C_ADDR);
    }

    // 3. Diagnostic Actionneur Haptique (Vibreur PWM sur GPIO 7)
    Serial.printf("[HAPTIC] Initialisation PWM sur GPIO %d (Fréq: %d Hz)...\n", PIN_HAPTIC_PWM, HAPTIC_LEDC_FREQ_HZ);
    haptic.begin(PIN_HAPTIC_PWM);
    Serial.println("[HAPTIC] Exécution du test de montée en puissance (Ramp-up)...");
    haptic.testRampUp();
    Serial.println("[PASS] Test PWM haptique terminé.");

    Serial.println("========================================================\n");
}

void setup() {
    // Initialisation moniteur série USB natif CDC
    Serial.begin(115200);
    delay(1000); // Laisse le temps au port USB CDC de s'énumérer

    // Configuration du bouton d'appairage sur GPIO 14
    pinMode(PIN_BTN_PAIRING, INPUT_PULLUP);

    // Configuration du réveil Deep Sleep (ext1 sur GPIO 14 actif bas)
    CONFIGURE_EXT1_WAKEUP();

    // Diagnostics matériels initiaux
    runHardwareDiagnostics();

    // Configuration des callbacks d'interopérabilité BLE
    bleServer.setHapticCallback([](uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
        haptic.trigger(pattern, intensity, durationMs);
    });

    bleServer.setStudioCommandCallback([](const String& command) {
        Serial.printf("[STUDIO] Commande reçue : \"%s\"\n", command.c_str());
        if (command.startsWith("START")) {
            bleServer.notifyStudioControl("RECORDING 5.0");
        } else if (command == "CANCEL") {
            bleServer.notifyStudioControl("CANCELLED");
        }
    });

    // Démarrage du serveur NimBLE
    Serial.printf("[BLE] Démarrage du serveur NimBLE sous le nom \"%s\"...\n", BLE_DEVICE_NAME);
    bleServer.begin(BLE_DEVICE_NAME);
}

void loop() {
    uint32_t now = millis();

    // 1. Mise à jour de l'actionneur haptique
    haptic.update();

    // 2. Acquisition IMU à cadence stricte de 50 Hz (toutes les 20 ms)
    if (now - lastImuReadMs >= IMU_SAMPLE_PERIOD_MS) {
        lastImuReadMs = now;

        if (imuReady) {
            ImuRawFrame frame;
            if (imu.readFrame(frame, (uint16_t)(now & 0xFFFF))) {
                // En mode connecté nominal, on peut périodiquement afficher l'état
            }
        }
    }

    // 3. Affichage périodique de télémétrie de debug (toutes les 3 secondes)
    if (now - lastDiagnosticPrintMs >= 3000) {
        lastDiagnosticPrintMs = now;

        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        if (imuReady) {
            imu.readRawMetrics(ax, ay, az, gx, gy, gz);
        }

        Serial.printf("[HEARTBEAT] BLE: %s | MTU: %d | IMU: [%.2f, %.2f, %.2f] g | PSRAM Libre: %u Ko\n",
                      bleServer.isConnected() ? "CONNECTÉ" : "ADVERTISING",
                      bleServer.getNegotiatedMtu(),
                      ax, ay, az,
                      ESP.getFreePsram() / 1024);
    }

    // 4. Gestion de l'inactivité et bascule en Deep Sleep après 3 minutes sans connexion
    if (!bleServer.isConnected()) {
        uint32_t inactiveMs = now - bleServer.getLastActivityTime();
        if (inactiveMs >= DEEP_SLEEP_TIMEOUT_MS) {
            Serial.printf("[POWER] Inactivité de %u secondes atteinte. Extinction en Deep Sleep...\n", BLE_ADVERTISING_TIMEOUT_SEC);
            Serial.println("[POWER] Appuyer sur le bouton GPIO 14 pour réveiller la chaussure.");
            Serial.flush();

            bleServer.stopAdvertising();
            haptic.stop();

            // Entrée en sommeil profond
            esp_deep_sleep_start();
        }
    }

    // 5. Lecture du bouton poussoir (Appui court pour forcer ré-annonce ou test haptique)
    if (digitalRead(PIN_BTN_PAIRING) == LOW) {
        delay(50); // Anti-rebond
        if (digitalRead(PIN_BTN_PAIRING) == LOW) {
            Serial.println("[BTN] Bouton GPIO 14 appuyé : Déclenchement impulsion test 100ms");
            haptic.trigger(HAPTIC_PATTERN_CONTINUOUS, 180, 100);
            while (digitalRead(PIN_BTN_PAIRING) == LOW) {
                delay(10);
            }
        }
    }
}
