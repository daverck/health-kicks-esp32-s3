#include "haptic_driver.h"
#include <Arduino.h>

static TaskHandle_t s_haptic_task_handle = nullptr;
static volatile uint16_t s_pending_duration = 0;
static volatile uint8_t s_pending_intensity = 0;

static void haptic_task(void* pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.printf("[HAPTIC] Activation GPIO %d pour %d ms (val=%d)\n", 
                      PIN_HAPTIC_PWM, s_pending_duration, s_pending_intensity);

        // Pilotage direct à l'état HAUT
        digitalWrite(PIN_HAPTIC_PWM, HIGH);
        vTaskDelay(pdMS_TO_TICKS(s_pending_duration));
        digitalWrite(PIN_HAPTIC_PWM, LOW);

        Serial.println("[HAPTIC] Extinction GPIO terminee");
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
    Serial.printf("[HAPTIC] Driver initialise en pilotage direct GPIO %d (FreeRTOS Task)\n", _pin);
}

void HapticDriver::setIntensity(uint8_t intensity) {
    // Pilotage direct tout-ou-rien
}

void HapticDriver::play(uint8_t pattern, uint8_t intensity, uint16_t duration_ms) {
    if (duration_ms == 0 || intensity == 0) {
        stop();
        return;
    }
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
    // Rien a faire dans loop()
}

void HapticDriver::testRampUp() {
    play(0, 255, 300);
}
