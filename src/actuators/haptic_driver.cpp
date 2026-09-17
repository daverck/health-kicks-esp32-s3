#include "haptic_driver.h"
#include <esp_arduino_version.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define HAPTIC_LEDC_ATTACH(pin, freq, res) ledcAttach(pin, freq, res)
#define HAPTIC_LEDC_WRITE(pin, val)        ledcWrite(pin, val)
#else
#define HAPTIC_LEDC_ATTACH(pin, freq, res) do { \
    ledcSetup(HAPTIC_LEDC_CHANNEL, freq, res); \
    ledcAttachPin(pin, HAPTIC_LEDC_CHANNEL); \
} while (0)
#define HAPTIC_LEDC_WRITE(pin, val)        ledcWrite(HAPTIC_LEDC_CHANNEL, val)
#endif

static uint32_t s_stop_time = 0;
static bool s_is_active = false;

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

    // Forcer la broche en sortie à l'état BAS avant LEDC (anti-glitch au boot)
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    HAPTIC_LEDC_ATTACH(_pin, 10000, 8); // GPIO 7, 10 kHz, 8 bits
    HAPTIC_LEDC_WRITE(_pin, 0);

    s_is_active = false;
    _isActive = false;
    _currentIntensity = 0;
    _stopTimestampMs = 0;
    _patternStep = 0;

    Serial.printf("[HAPTIC] INIT: GPIO %d configure (10 kHz, 8 bits, canal %d)\n",
                  _pin, HAPTIC_LEDC_CHANNEL);
}

void HapticDriver::setIntensity(uint8_t intensity) {
    HAPTIC_LEDC_WRITE(_pin, intensity);
}

void HapticDriver::play(uint8_t pattern, uint8_t intensity, uint16_t duration_ms) {
    if (intensity == 0 || duration_ms == 0) {
        stop();
        return;
    }

    s_is_active = true;
    _isActive = true;
    _pattern = pattern;
    _currentIntensity = intensity;
    s_stop_time = millis() + duration_ms;
    _stopTimestampMs = s_stop_time;

    HAPTIC_LEDC_WRITE(_pin, intensity);
    Serial.printf("[HAPTIC] START: intensity=%d, duration=%d ms (stop a %u)\n", intensity, duration_ms, s_stop_time);
}

void HapticDriver::update() {
    if (s_is_active) {
        if ((long)(millis() - s_stop_time) >= 0) {
            stop();
            Serial.println("[HAPTIC] STOP: fin vibration");
        }
    }
}

void HapticDriver::stop() {
    s_is_active = false;
    _isActive = false;
    _currentIntensity = 0;
    HAPTIC_LEDC_WRITE(_pin, 0);
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
