#pragma once

#include <Arduino.h>
#include "config.h"
#include "invariants.h"

class HapticDriver {
public:
    HapticDriver();

    /**
     * @brief Initializes GPIO to LOW state before attaching the LEDC PWM channel (anti-glitch on boot).
     */
    void init(int pin = PIN_HAPTIC_PWM);
    void begin(int pin = PIN_HAPTIC_PWM) { init(pin); }

    /**
     * @brief Triggers a haptic pattern.
     * @param pattern 0 = continuous, 1 = double pulse, 2 = alert pulsation.
     * @param intensity PWM intensity (0-255).
     * @param durationMs Total duration of stimulation in milliseconds.
     */
    void play(uint8_t pattern, uint8_t intensity, uint16_t durationMs);
    void trigger(uint8_t pattern, uint8_t intensity, uint16_t durationMs) {
        play(pattern, intensity, durationMs);
    }

    /**
     * @brief Directly applies continuous PWM intensity.
     */
    void setIntensity(uint8_t intensity);

    /**
     * @brief Immediately stops vibration and resets state.
     */
    void stop();

    /**
     * @brief Non-blocking update loop based on millis() for automatic timeout/stop.
     */
    void update();

    /**
     * @brief Progressive ramp-up test routine on demand.
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
