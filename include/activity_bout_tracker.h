#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <vector>
#include "invariants.h"

/**
 * @file activity_bout_tracker.h
 * @brief Autonomous Activity Bout & Step Ventilation Tracker for ESP32-S3.
 *
 * Tracks 100% of physical movement as self-contained Activity Bouts in a 128-entry RAM ring buffer.
 * Opens on stride initiation (>= 2 steps), finalizes after 10 seconds of stillness or activity transition,
 * and enables reverse-time calibration on BLE reconnection.
 */
class ActivityBoutTracker {
public:
    static constexpr size_t MAX_BOUTS = 128;
    static constexpr uint32_t STILLNESS_TIMEOUT_SEC = 10;

    ActivityBoutTracker();

    /**
     * @brief Initializes the activity bout tracker.
     */
    void begin();

    /**
     * @brief Ingests a validated step and updates active bout state.
     * @param activityState Current ML activity state code (e.g., STATE_CODE_WALK, STATE_CODE_RUN).
     * @param nowMs Current system timestamp in ms.
     * @param cadence Instantaneous cadence in steps per minute (SPM).
     */
    void onStep(uint8_t activityState, uint32_t nowMs, uint8_t cadence);

    /**
     * @brief Periodic tick to evaluate stillness timeout and close idle bouts.
     * @param nowMs Current system timestamp in ms.
     */
    void onTick(uint32_t nowMs);

    /**
     * @brief Returns the count of unacknowledged bouts stored in the buffer.
     */
    size_t getBoutCount() const { return _count; }

    /**
     * @brief Returns whether an active bout is currently open.
     */
    bool isActiveBoutOpen() const { return _activeBoutOpen; }

    /**
     * @brief Returns a copy of the unacknowledged bouts in chronological order.
     * @param maxBouts Maximum number of bouts to fetch.
     * @param outBouts Output vector to populate.
     */
    void getBouts(size_t maxBouts, std::vector<ActivityBout>& outBouts);

    /**
     * @brief Advances the buffer read index past acknowledged bouts.
     * @param count Number of bouts acknowledged by client.
     */
    void acknowledgeCount(uint8_t count);

    /**
     * @brief Clears all stored bouts and active bout state.
     */
    void reset();

    /**
     * @brief Flushes/finalizes the currently active bout if open.
     * @param nowMs Current system timestamp in ms.
     */
    void flushActiveBout(uint32_t nowMs);

    /**
     * @brief Helper to map raw state code to canonical activity type (0x01 Walk, 0x02 Run, 0x03 Stairs, 0x04 Unclassified).
     */
    static uint8_t normalizeActivityType(uint8_t stateCode);

private:
    void finalizeCurrentBout(uint32_t endUptimeSec);

    ActivityBout _bouts[MAX_BOUTS];
    size_t _head;
    size_t _tail;
    size_t _count;

    bool _activeBoutOpen;
    uint32_t _activeStartUptimeSec;
    uint32_t _activeLastStepUptimeSec;
    uint16_t _activeStepCount;
    uint8_t _activeActivityType;
    uint32_t _activeCadenceSum;
    uint16_t _activeCadenceSamples;
};
