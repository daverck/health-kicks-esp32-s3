#include "haptic_driver.h"
#include <Arduino.h>

// Utilisation explicite du canal 4 sur Arduino core v2.x
#define HAPTIC_CHANNEL 4
#define HAPTIC_FREQ    10000
#define HAPTIC_RES     8

static uint32_t s_stop_time = 0;
static bool s_is_active = false;
static uint8_t s_pin = PIN_HAPTIC_PWM;

HapticDriver::HapticDriver()
    : _pin(PIN_HAPTIC_PWM),
      _pattern(HAPTIC_PATTERN_CONTINUOUS),
      _currentIntensity(0),
      _stopTimestampMs(0),
      _isActive(false),
      _patternStep(0),
      _stepTime(0) {}

void HapticDriver::init(int pin) {
    s_pin = pin;
    _pin = pin;
    s_is_active = false;
    _isActive = false;

    // 1. Initialiser le canal LEDC
    ledcSetup(HAPTIC_CHANNEL, HAPTIC_FREQ, HAPTIC_RES);
    // 2. Attacher la broche physique au canal
    ledcAttachPin(s_pin, HAPTIC_CHANNEL);
    // 3. Forcer la valeur à 0
    ledcWrite(HAPTIC_CHANNEL, 0);

    Serial.printf("[HAPTIC] INIT: GPIO %d attache au canal LEDC %d (10 kHz, 8 bits)\n", s_pin, HAPTIC_CHANNEL);
}

void HapticDriver::setIntensity(uint8_t intensity) {
    ledcWrite(HAPTIC_CHANNEL, intensity);
}

void HapticDriver::play(uint8_t pattern, uint8_t intensity, uint16_t duration_ms) {
    if (intensity == 0 || duration_ms == 0) {
        stop();
        return;
    }

    s_is_active = true;
    _isActive = true;
    _currentIntensity = intensity;
    s_stop_time = millis() + duration_ms;

    ledcWrite(HAPTIC_CHANNEL, intensity);
    Serial.printf("[HAPTIC] START: val=%d sur canal %d, duree=%d ms (stop a %u)\n", 
                  intensity, HAPTIC_CHANNEL, duration_ms, s_stop_time);
}

void HapticDriver::update() {
    if (s_is_active) {
        if ((long)(millis() - s_stop_time) >= 0) {
            stop();
            Serial.println("[HAPTIC] STOP: extinction");
        }
    }
}

void HapticDriver::stop() {
    s_is_active = false;
    _isActive = false;
    _currentIntensity = 0;
    ledcWrite(HAPTIC_CHANNEL, 0);
}

void HapticDriver::testRampUp() {
    Serial.println("--- Démarrage Test Ramp-Up Vibreur Haptique (0 -> 255) ---");
    for (int i = 0; i <= 255; i += 15) {
        setIntensity(i);
        delay(30);
    }
    delay(200);
    for (int i = 255; i >= 0; i -= 15) {
        setIntensity(i);
        delay(30);
    }
    stop();
    Serial.println("--- Fin Test Ramp-Up Vibreur Haptique ---");
}
