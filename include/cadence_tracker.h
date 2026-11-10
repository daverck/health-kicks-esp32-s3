#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/**
 * @file cadence_tracker.h
 * @brief Sliding-window cadence (steps per minute) rate tracker for ESP32-S3.
 */

class CadenceTracker {
public:
    static constexpr uint32_t DEFAULT_CADENCE_WINDOW_MS = 5000;  // 5.0 seconds sliding window
    static constexpr uint32_t DEFAULT_CADENCE_TIMEOUT_MS = 3000; // Zero cadence if no step within 3.0s
    static constexpr size_t MAX_CADENCE_HISTORY = 32;            // Ring buffer capacity

    explicit CadenceTracker(uint32_t windowMs = DEFAULT_CADENCE_WINDOW_MS,
                            uint32_t timeoutMs = DEFAULT_CADENCE_TIMEOUT_MS);

    /**
     * @brief Records a newly detected step timestamp into the ring buffer.
     * @param timestampMs System timestamp of detected step in ms.
     */
    void addStepTimestamp(uint32_t timestampMs);

    /**
     * @brief Calculates current cadence in steps per minute (SPM).
     * @param nowMs Current system timestamp in ms.
     * @param lastStepTimeMs Timestamp of last recorded step in ms.
     * @return Cadence in SPM (0 to 255).
     */
    uint8_t calculateCadence(uint32_t nowMs, uint32_t lastStepTimeMs);

    /**
     * @brief Drops timestamps older than the sliding window from the ring buffer.
     * @param nowMs Current system timestamp in ms.
     */
    void cleanCadenceBuffer(uint32_t nowMs);

    /**
     * @brief Resets the ring buffer and counters.
     */
    void reset();

    /**
     * @brief Computes cadence in steps per minute (SPM) based on step interval count and duration in ms.
     * @param intervalCount Number of step intervals between oldest and newest step (count - 1).
     * @param deltaMs Duration in ms between oldest and newest step timestamp.
     * @return Cadence in SPM (0 to 255).
     */
    static uint8_t computeStepsPerMinute(size_t intervalCount, uint32_t deltaMs);

    // Getters for inspection/unit testing
    size_t getTimestampCount() const { return _timestampCount; }
    uint32_t getWindowMs() const { return _windowMs; }
    uint32_t getTimeoutMs() const { return _timeoutMs; }

private:
    size_t getOldestIndex() const;
    size_t getNewestIndex() const;

    uint32_t _windowMs;
    uint32_t _timeoutMs;
    uint32_t _stepTimestamps[MAX_CADENCE_HISTORY];
    size_t _timestampHead;
    size_t _timestampCount;
};
