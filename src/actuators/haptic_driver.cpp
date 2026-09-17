#include "haptic_driver.h"
#include <esp_arduino_version.h>

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

    // 1. Correctif Glitch au Boot : forcer la broche en sortie à l'état BAS avant LEDC
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    // 2. Attachement du canal PWM LEDC et valeur initiale à 0
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(_pin, HAPTIC_LEDC_FREQ_HZ, HAPTIC_LEDC_RES_BITS);
    ledcWrite(_pin, 0);
#else
    ledcSetup(HAPTIC_LEDC_CHANNEL, HAPTIC_LEDC_FREQ_HZ, HAPTIC_LEDC_RES_BITS);
    ledcAttachPin(_pin, HAPTIC_LEDC_CHANNEL);
    ledcWrite(HAPTIC_LEDC_CHANNEL, 0);
#endif

    _isActive = false;
    _currentIntensity = 0;
    _stopTimestampMs = 0;
    _patternStep = 0;

    log_i("Driver haptique PWM initialisé sur GPIO %d (anti-glitch LOW, Fréq: %d Hz)", _pin, HAPTIC_LEDC_FREQ_HZ);
}

void HapticDriver::setIntensity(uint8_t intensity) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(_pin, intensity);
#else
    ledcWrite(HAPTIC_LEDC_CHANNEL, intensity);
#endif
}

void HapticDriver::play(uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
    if (intensity == 0 || durationMs == 0) {
        stop();
        return;
    }

    _pattern = pattern;
    _currentIntensity = intensity;
    _stopTimestampMs = millis() + durationMs;
    _patternStep = 0;
    _stepTime = millis();
    _isActive = true;

    setIntensity(_currentIntensity);
    Serial.printf("[HAPTIC] Play: intensity=%d, duration=%d ms\n", intensity, durationMs);
}

void HapticDriver::stop() {
    setIntensity(0);
    _isActive = false;
    _currentIntensity = 0;
    _patternStep = 0;
}

void HapticDriver::update() {
    if (!_isActive) {
        return;
    }

    // Arrêt strict dès que le délai est écoulé (sécurisé contre le rollover millis)
    if ((long)(millis() - _stopTimestampMs) >= 0) {
        stop();
        return;
    }

    uint32_t now = millis();

    // Gestion des motifs avancés
    switch (_pattern) {
        case HAPTIC_PATTERN_CONTINUOUS:
            break;

        case HAPTIC_PATTERN_DOUBLE_PULSE:
            if (_patternStep == 0 && (now - _stepTime) >= 120) {
                setIntensity(0);
                _patternStep = 1;
                _stepTime = now;
            } else if (_patternStep == 1 && (now - _stepTime) >= 100) {
                setIntensity(_currentIntensity);
                _patternStep = 2;
                _stepTime = now;
            } else if (_patternStep == 2 && (now - _stepTime) >= 120) {
                setIntensity(0);
                _patternStep = 3;
                _stepTime = now;
            }
            break;

        case HAPTIC_PATTERN_ALERT_PULSE:
            if (((now - _stepTime) / 80) % 2 == 0) {
                setIntensity(_currentIntensity);
            } else {
                setIntensity(0);
            }
            break;

        default:
            break;
    }
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
