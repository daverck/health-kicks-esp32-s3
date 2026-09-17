#include "haptic_driver.h"
#include <esp_arduino_version.h>

HapticDriver::HapticDriver()
    : _pin(PIN_HAPTIC_PWM),
      _pattern(HAPTIC_PATTERN_CONTINUOUS),
      _intensity(0),
      _durationMs(0),
      _startTime(0),
      _stopTime(0),
      _isVibrating(false),
      _patternStep(0),
      _stepTime(0) {}

void HapticDriver::begin(int pin) {
    _pin = pin;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(_pin, HAPTIC_LEDC_FREQ_HZ, HAPTIC_LEDC_RES_BITS);
    ledcWrite(_pin, 0);
#else
    ledcSetup(HAPTIC_LEDC_CHANNEL, HAPTIC_LEDC_FREQ_HZ, HAPTIC_LEDC_RES_BITS);
    ledcAttachPin(_pin, HAPTIC_LEDC_CHANNEL);
    ledcWrite(HAPTIC_LEDC_CHANNEL, 0);
#endif

    _isVibrating = false;
    log_i("Driver haptique PWM initialisé sur GPIO %d (Fréq: %d Hz, Rés: %d bits)", _pin, HAPTIC_LEDC_FREQ_HZ, HAPTIC_LEDC_RES_BITS);
}

void HapticDriver::setIntensity(uint8_t intensity) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(_pin, intensity);
#else
    ledcWrite(HAPTIC_LEDC_CHANNEL, intensity);
#endif
}

void HapticDriver::trigger(uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
    if (intensity == 0 || durationMs == 0) {
        stop();
        return;
    }

    _pattern = pattern;
    _intensity = intensity;
    _durationMs = durationMs;
    _startTime = millis();
    _stopTime = _startTime + durationMs;
    _patternStep = 0;
    _stepTime = _startTime;
    _isVibrating = true;

    setIntensity(_intensity);
    log_i("Déclenchement haptique: Pattern=%d, Intensité=%d/255, Durée=%d ms", pattern, intensity, durationMs);
}

void HapticDriver::stop() {
    setIntensity(0);
    _isVibrating = false;
    _patternStep = 0;
}

void HapticDriver::update() {
    if (!_isVibrating) {
        return;
    }

    uint32_t now = millis();

    // Arrêt global si la durée est expirée
    if (now >= _stopTime) {
        stop();
        return;
    }

    // Gestion des motifs vibratoires avancés
    switch (_pattern) {
        case HAPTIC_PATTERN_CONTINUOUS:
            // Reste à _intensity jusqu'à expiration de _stopTime
            break;

        case HAPTIC_PATTERN_DOUBLE_PULSE:
            // Cycle : Pulse (120ms) -> Pause (100ms) -> Pulse (120ms) -> Pause...
            if (_patternStep == 0 && (now - _stepTime) >= 120) {
                setIntensity(0);
                _patternStep = 1;
                _stepTime = now;
            } else if (_patternStep == 1 && (now - _stepTime) >= 100) {
                setIntensity(_intensity);
                _patternStep = 2;
                _stepTime = now;
            } else if (_patternStep == 2 && (now - _stepTime) >= 120) {
                setIntensity(0);
                _patternStep = 3;
                _stepTime = now;
            }
            break;

        case HAPTIC_PATTERN_ALERT_PULSE:
            // Pulsation rapide 80ms ON / 80ms OFF
            if ((now / 80) % 2 == 0) {
                setIntensity(_intensity);
            } else {
                setIntensity(0);
            }
            break;

        default:
            break;
    }
}

void HapticDriver::testRampUp() {
    log_i("--- Démarrage Test Ramp-Up Vibreur Haptique (0 -> 255) ---");
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
    log_i("--- Fin Test Ramp-Up Vibreur Haptique ---");
}
