#pragma once

#include <Arduino.h>
#include "config.h"
#include "invariants.h"

class HapticDriver {
public:
    HapticDriver();

    /**
     * @brief Initialise le GPIO à l'état BAS avant d'attacher le canal LEDC PWM (anti-glitch au boot).
     */
    void init(int pin = PIN_HAPTIC_PWM);
    void begin(int pin = PIN_HAPTIC_PWM) { init(pin); }

    /**
     * @brief Déclenche un motif haptique.
     * @param pattern 0 = continu, 1 = double pulse, 2 = pulsation alerte.
     * @param intensity Intensité PWM (0-255).
     * @param durationMs Durée totale de la stimulation en millisecondes.
     */
    void play(uint8_t pattern, uint8_t intensity, uint16_t durationMs);
    void trigger(uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
        play(pattern, intensity, durationMs);
    }

    /**
     * @brief Applique directement une intensité PWM continue.
     */
    void setIntensity(uint8_t intensity);

    /**
     * @brief Coupe immédiatement le vibreur et réinitialise l'état.
     */
    void stop();

    /**
     * @brief Boucle de mise à jour non-bloquante basée sur millis() pour extinction automatique.
     */
    void update();

    /**
     * @brief Routine de test progressif (Ramp-up) sur demande.
     */
    void testRampUp();

    bool isVibrating() const { return _isActive; }

private:
    int _pin;
    uint8_t _pattern;
    uint8_t _currentIntensity;
    uint32_t _stopTimestampMs;
    bool _isActive;

    uint8_t _patternStep;
    uint32_t _stepTime;
};
