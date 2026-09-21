#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "invariants.h"

/**
 * @file step_detector.h
 * @brief Autonomous Deterministic Pedometer & Cadence Estimation Engine for ESP32-S3.
 */

class StepDetector {
public:
    static constexpr float DEFAULT_STEP_THRESHOLD_G = 0.35f;  // a_vert threshold above 1.0g (Az > 1.35g)
    static constexpr uint32_t DEFAULT_REFRACTORY_MS = 260;    // Minimum refractory / debounce time between steps
    static constexpr uint32_t CADENCE_WINDOW_MS = 5000;       // 5.0 seconds sliding window for cadence
    static constexpr uint32_t CADENCE_TIMEOUT_MS = 3000;      // Zero cadence if no step within 3.0 seconds
    static constexpr size_t MAX_CADENCE_HISTORY = 32;         // Maximum step timestamps in ring buffer

    StepDetector();

    /**
     * @brief Initializes step detector parameters and counters.
     * @param stepThreshold Dynamic vertical acceleration threshold in g (default: 0.35g).
     * @param refractoryMs Debounce refractory period in ms (default: 260 ms).
     */
    void begin(float stepThreshold = DEFAULT_STEP_THRESHOLD_G, uint32_t refractoryMs = DEFAULT_REFRACTORY_MS);

    /**
     * @brief Ingests a calibrated IMU acceleration sample and evaluates step detection.
     * @param ax Calibrated X acceleration in g.
     * @param ay Calibrated Y acceleration in g.
     * @param az Calibrated Z acceleration in g (vertical resting norm = 1.0g).
     * @param currentActivityState Current ML activity state code (e.g., STATE_CODE_WALK).
     * @param nowMs Current system timestamp in ms.
     * @return true if a valid new step was detected and counted, false otherwise.
     */
    bool processSample(float ax, float ay, float az, uint8_t currentActivityState, uint32_t nowMs);

    /**
     * @brief Computes instant cadence in Steps Per Minute (SPM) based on 5s sliding window.
     * @param nowMs Current system timestamp in ms.
     * @return Cadence in SPM (0 to 255).
     */
    uint8_t calculateCadence(uint32_t nowMs);

    /**
     * @brief Generates the big-endian packed StepCounterPayload for BLE transmission.
     * @param nowMs Current system timestamp in ms.
     * @return StepCounterPayload with network big-endian encoded values.
     */
    StepCounterPayload getPayload(uint32_t nowMs);

    // Getters
    uint32_t getTotalSteps() const { return _totalSteps; }
    uint16_t getWalkSteps() const { return _walkSteps; }
    uint16_t getRunSteps() const { return _runSteps; }
    uint16_t getStairsSteps() const { return _stairsSteps; }
    uint16_t getUnclassifiedSteps() const { return _unclassifiedSteps; }
    uint32_t getLastStepTimeMs() const { return _lastStepTimeMs; }

    /**
     * @brief Resets all step accumulators and cadence history.
     */
    void reset();

private:
    float _stepThreshold;
    uint32_t _refractoryMs;

    // Step state machine
    enum class StepState {
        ARMED,
        PEAK_DETECTED
    };
    StepState _state;

    uint32_t _totalSteps;
    uint16_t _walkSteps;
    uint16_t _runSteps;
    uint16_t _stairsSteps;
    uint16_t _unclassifiedSteps;

    uint32_t _lastStepTimeMs;
    float _peakVertAccel;

    // Ring buffer for cadence sliding window
    uint32_t _stepTimestamps[MAX_CADENCE_HISTORY];
    size_t _timestampHead;
    size_t _timestampCount;

    void addStepTimestamp(uint32_t timestampMs);
    void cleanCadenceBuffer(uint32_t nowMs);
};
