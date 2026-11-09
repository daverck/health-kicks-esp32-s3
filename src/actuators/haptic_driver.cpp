#include "haptic_driver.h"
#include <Arduino.h>

static TaskHandle_t s_haptic_task_handle = nullptr;
static volatile uint8_t s_pending_pattern = 0;
static volatile uint16_t s_pending_duration = 0;
static volatile uint8_t s_pending_intensity = 0;

static void executeDoublePulse(int pin) {
    // Double pulse: 2 crisp distinct vibration pulses with inter-pulse pause
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(220));
    digitalWrite(pin, LOW);
    vTaskDelay(pdMS_TO_TICKS(140));
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(220));
    digitalWrite(pin, LOW);
}

static void executeAlertPulse(int pin) {
    // Triple alert pulses
    for (int i = 0; i < 3; i++) {
        digitalWrite(pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(180));
        digitalWrite(pin, LOW);
        if (i < 2) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void executeContinuousDrive(int pin, uint16_t durationMs) {
    // Direct continuous drive for requested duration
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(durationMs > 0 ? durationMs : 200));
    digitalWrite(pin, LOW);
}

static void haptic_task(void* pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.printf("[HAPTIC] Activating GPIO %d (pattern=%u, dur=%d ms, val=%d)\n", 
                      PIN_HAPTIC_PWM, s_pending_pattern, s_pending_duration, s_pending_intensity);

        switch (s_pending_pattern) {
            case HAPTIC_PATTERN_DOUBLE_PULSE:
                executeDoublePulse(PIN_HAPTIC_PWM);
                break;
            case HAPTIC_PATTERN_ALERT_PULSE:
                executeAlertPulse(PIN_HAPTIC_PWM);
                break;
            case HAPTIC_PATTERN_CONTINUOUS:
            default:
                executeContinuousDrive(PIN_HAPTIC_PWM, s_pending_duration);
                break;
        }

        Serial.println("[HAPTIC] Vibration completed, GPIO LOW");
    }
}

HapticDriver::HapticDriver()
    : _pin(PIN_HAPTIC_PWM),
      _pattern(HAPTIC_PATTERN_CONTINUOUS),
      _currentIntensity(0),
      _stopTimestampMs(0),
      _isActive(false),
      _patternStep(0),
      _stepTime(0) {}

void HapticDriver::init(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    if (s_haptic_task_handle == nullptr) {
        xTaskCreate(
            haptic_task,
            "haptic_task",
            2048,
            nullptr,
            configMAX_PRIORITIES - 2,
            &s_haptic_task_handle
        );
    }
    Serial.printf("[HAPTIC] Driver initialized with direct GPIO drive %d (FreeRTOS Task)\n", _pin);
}

void HapticDriver::setIntensity(uint8_t intensity) {
    // Direct digital drive
}

void HapticDriver::play(uint8_t pattern, uint8_t intensity, uint16_t duration_ms) {
    if (intensity == 0) {
        stop();
        return;
    }
    s_pending_pattern = pattern;
    s_pending_intensity = intensity;
    s_pending_duration = duration_ms;

    if (s_haptic_task_handle != nullptr) {
        xTaskNotifyGive(s_haptic_task_handle);
    }
}

void HapticDriver::stop() {
    digitalWrite(_pin, LOW);
}

void HapticDriver::update() {
    // Handled by FreeRTOS task
}

void HapticDriver::testRampUp() {
    play(HAPTIC_PATTERN_DOUBLE_PULSE, 255, 400);
}
