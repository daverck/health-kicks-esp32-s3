#pragma once

#include <Arduino.h>
#include "esp_sleep.h"

/**
 * @file config.h
 * @brief Assignations matérielles, contraintes d'architecture et constantes pour ESP32-S3-N16R8.
 */

// ============================================================================
// 1. ASSIGNATION STRICTE DES BROCHES (PINOUT ESP32-S3-N16R8)
// ============================================================================

/**
 * @note ATTENTION CRITIQUE OCTAL FLASH & PSRAM (N16R8) :
 * Les broches GPIO 33, 34, 35, 36, 37 sont physiquement connectées aux bus
 * SPI haute vitesse de la mémoire Flash Octal et de la PSRAM Octal.
 * IL EST STRICTEMENT INTERDIT DE LES UTILISER EN TANT QUE GPIO GÉNÉRAUX.
 *
 * Broches Strapping à ne pas tirer à l'état haut/bas au boot : GPIO 0, 45, 46.
 * Broches USB natif JTAG/CDC : GPIO 19 (D-), GPIO 20 (D+).
 */

// Bus I2C pour IMU (MPU-6050)
#define PIN_IMU_SDA             GPIO_NUM_4   // RTC IO 4 (Pull-up 4.7k vers 3.3V)
#define PIN_IMU_SCL             GPIO_NUM_5   // RTC IO 5 (Pull-up 4.7k vers 3.3V)
#define PIN_IMU_INT             GPIO_NUM_6   // Interruption matérielle IMU (optionnelle)

/**
 * Actionneur Haptique (Vibreur)
 * @note Recommandation Hardware : Utiliser un N-MOSFET à très bas seuil de déclenchement
 * (Logic-Level Vgs(th) < 1.8V, idéalement AO3400) avec résistance de gate 100 Ohms,
 * pull-down 100k Ohms et diode de roue libre (1N4148 / Schottky) aux bornes du moteur.
 */
#define PIN_HAPTIC_PWM          GPIO_NUM_7   // Sortie LEDC PWM

// Bouton Poussoir (Appairage BLE & Réveil Deep Sleep)
#define PIN_BTN_PAIRING         GPIO_NUM_14  // RTC IO 14, actif à l'état bas (GND)

// LED d'état RGB adressable (DevKitC-1 embarquée)
#define PIN_LED_RGB             GPIO_NUM_48  // WS2812 Data In

// Mesure analogique de la tension batterie
#define PIN_VBAT_SENSE          GPIO_NUM_10  // ADC1_CHANNEL_9 (Pont diviseur 2x 100k)

// ============================================================================
// 2. CONSTANTES LOGICIELLES & BUS
// ============================================================================

// Paramètres I2C IMU
#define IMU_I2C_ADDR            0x68
#define IMU_I2C_FREQ_HZ         400000       // I2C Fast Mode (400 kHz)
#define IMU_SAMPLE_RATE_HZ      50           // Cadence nominale 50 Hz
#define IMU_SAMPLE_PERIOD_MS    (1000 / IMU_SAMPLE_RATE_HZ) // 20 ms

// Paramètres LEDC PWM pour le vibreur
#define HAPTIC_LEDC_CHANNEL     0
#define HAPTIC_LEDC_TIMER       0
#define HAPTIC_LEDC_FREQ_HZ     10000        // Fréquence PWM 10 kHz
#define HAPTIC_LEDC_RES_BITS    8            // Résolution 8 bits (0 - 255)

// Paramètres BLE
#define BLE_DEVICE_NAME         "HealthKicks-HK-2"
#define BLE_ADVERTISING_TIMEOUT_SEC 180      // 3 minutes sans connexion avant Deep Sleep
#define BLE_PREFERRED_MTU       247

// Configuration Deep Sleep
#define DEEP_SLEEP_TIMEOUT_MS   (BLE_ADVERTISING_TIMEOUT_SEC * 1000UL)

/**
 * @brief Macro de configuration du réveil Deep Sleep via le bouton GPIO 14 (ext1)
 */
#define CONFIGURE_EXT1_WAKEUP() \
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_BTN_PAIRING, ESP_EXT1_WAKEUP_ANY_LOW)

