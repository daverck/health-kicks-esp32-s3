#pragma once

#include <Arduino.h>
#include "config.h"
#include "invariants.h"

class HapticDriver {
public:
    HapticDriver();

    /**
     * @brief Initialise le canal LEDC PWM sur la broche du vibreur (GPIO 7).
     */
    void begin(int pin = PIN_HAPTIC_PWM);

    /**
     * @brief Applique directement une intensité PWM continue.
     * @param intensity Valeur entre 0 (arrêt) et 255 (puissance max).
     */
    void setIntensity(uint8_t intensity);

    /**
     * @brief Déclenche un motif haptique conforme à contracts/ble_gatt_specs.md.
     * @param pattern 0 = continu, 1 = double pulse, 2 = pulsation alerte.
     * @param intensity Intensité PWM (0-255).
     * @param durationMs Durée totale de la stimulation en millisecondes.
     */
    void trigger(uint8_t pattern, uint8_t intensity, uint16_t durationMs);

    /**
     * @brief Coupe immédiatement le vibreur et réinitialise l'état.
     */
    void stop();

    /**
     * @brief Boucle de mise à jour non-bloquante du timer et des patterns.
     * À appeler périodiquement (ex: toutes les 10ms ou dans loop/task).
     */
    void update();

    /**
     * @brief Routine de test progressif (Ramp-up) pour validation matérielle.
     */
    void testRampUp();

    bool isVibrating() const { return _isVibrating; }

private:
    int _pin;
    uint8_t _pattern;
    uint8_t _intensity;
    uint16_t _durationMs;
    uint32_t _startTime;
    uint32_t _stopTime;
    bool _isVibrating;
    uint8_t _patternStep;
    uint32_t _stepTime;
};
