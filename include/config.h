#pragma once

#include <Arduino.h>
#include "esp_sleep.h"

/**
 * @file config.h
 * @brief Hardware pin assignments, architecture constraints, and constants for ESP32-S3-N16R8.
 */

// ============================================================================
// 1. STRICT PIN ASSIGNMENT (PINOUT ESP32-S3-N16R8)
// ============================================================================

/**
 * @note CRITICAL WARNING OCTAL FLASH & PSRAM (N16R8):
 * GPIO pins 33, 34, 35, 36, 37 are physically routed to the high-speed SPI
 * buses of the Octal Flash and Octal PSRAM.
 * THEY MUST NEVER BE USED AS GENERAL PURPOSE GPIOs.
 *
 * Strapping pins not to pull high/low at boot: GPIO 0, 45, 46.
 * Native USB JTAG/CDC pins: GPIO 19 (D-), GPIO 20 (D+).
 */

// I2C Bus for IMU (MPU-6050)
#define PIN_IMU_SDA             GPIO_NUM_4   // RTC IO 4 (4.7k Pull-up to 3.3V)
#define PIN_IMU_SCL             GPIO_NUM_5   // RTC IO 5 (4.7k Pull-up to 3.3V)
#define PIN_IMU_INT             GPIO_NUM_6   // IMU Hardware interrupt (optional)

/**
 * Haptic Actuator (Vibrator)
 * @note Hardware Recommendation: Use an N-MOSFET with very low threshold voltage
 * (Logic-Level Vgs(th) < 1.8V, ideally AO3400) with 100 Ohm gate resistor,
 * 100k Ohm pull-down, and flyback diode (1N4148 / Schottky) across motor terminals.
 */
#define PIN_HAPTIC_PWM          GPIO_NUM_7   // LEDC PWM output

// Push Button (BLE Pairing & Deep Sleep Wakeup)
#define PIN_BTN_PAIRING         GPIO_NUM_14  // RTC IO 14, active low (GND)

// Addressable RGB Status LED (Onboard DevKitC-1)
#define PIN_LED_RGB             GPIO_NUM_48  // WS2812 Data In

// Battery Voltage Analog Sense
#define PIN_VBAT_SENSE          GPIO_NUM_10  // ADC1_CHANNEL_9 (Voltage divider 2x 100k)

// ============================================================================
// 2. SOFTWARE CONSTANTS & BUS CONFIGURATION
// ============================================================================

// IMU I2C Parameters
#define IMU_I2C_ADDR            0x68
#define IMU_I2C_FREQ_HZ         400000       // I2C Fast Mode (400 kHz)
#define IMU_SAMPLE_FREQ_HZ      19           // Nominal rate 19 Hz (~94 samples for 5s)
#define IMU_SAMPLE_PERIOD_MS    53           // 53 ms period (5000 ms / 53 ms = ~94 samples)

// LEDC PWM Parameters for Haptic Motor
#define HAPTIC_LEDC_CHANNEL     0
#define HAPTIC_LEDC_TIMER       0
#define HAPTIC_LEDC_FREQ_HZ     10000        // PWM Frequency 10 kHz
#define HAPTIC_LEDC_RES_BITS    8            // 8-bit Resolution (0 - 255)

// BLE Parameters
#define BLE_DEVICE_NAME         "HealthKicks-HK-2"
#define BLE_ADVERTISING_TIMEOUT_SEC 180      // 3 minutes disconnected before Deep Sleep
#define BLE_PREFERRED_MTU       247

// Deep Sleep & Power Management Configuration
#define DEEP_SLEEP_INACTIVITY_TIMEOUT_SEC   900      // 15 minutes of stillness and disconnection before Deep Sleep
#define DEEP_SLEEP_TIMEOUT_MS               (DEEP_SLEEP_INACTIVITY_TIMEOUT_SEC * 1000UL)

// MPU-6050 Wake-On-Motion (WOM) Parameters
#define IMU_WOM_THRESHOLD                   20       // Motion threshold (1 LSB = 32 mg, 20 = ~640 mg)
#define IMU_WOM_DURATION                    2        // Minimum duration count for motion event

/**
 * @brief Configure Deep Sleep wakeup via GPIO 14 button (ext1)
 */
#define CONFIGURE_EXT1_WAKEUP() \
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_BTN_PAIRING, ESP_EXT1_WAKEUP_ANY_LOW)

/**
 * @brief Configure Deep Sleep wakeup via MPU-6050 INT pin on GPIO 6 (ext0)
 */
#define CONFIGURE_EXT0_WAKEUP() \
    esp_sleep_enable_ext0_wakeup(PIN_IMU_INT, 1)

