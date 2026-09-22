#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <Preferences.h>
#include <functional>
#include "invariants.h"

/**
 * @file inactivity_monitor.h
 * @brief Autonomous Prolonged Inactivity Reminder (Sedentary Reminder) Engine for ESP32-S3.
 * 
 * Tracks continuous stationary intervals, triggers discrete vibrotactile & BLE 0x20 alerts,
 * persists user preferences in NVS, and supports dynamic hot-reconfiguration.
 */

typedef std::function<void(uint32_t nowMs)> InactivityAlertCallback;

class InactivityMonitor {
public:
    static constexpr uint16_t DEFAULT_INACTIVITY_THRESHOLD_SEC = 3000; // 50 minutes
    static constexpr uint16_t DEFAULT_COOLDOWN_DURATION_SEC = 600;    // 10 minutes
    static constexpr uint32_t RESUME_STEP_THRESHOLD = 10;             // Steps required to clear alert

    InactivityMonitor();

    /**
     * @brief Initializes the inactivity monitor, loads preferences from NVS ("hk_inact").
     */
    void begin();

    /**
     * @brief Hot-reconfigures inactivity monitor parameters and persists them to NVS.
     * @param enabled Enable or disable inactivity monitoring.
     * @param inactivityThresholdSec Continuous inactivity threshold before alert (in seconds).
     * @param cooldownDurationSec Repeat alert cooldown interval (in seconds).
     */
    void configure(bool enabled, uint16_t inactivityThresholdSec, uint16_t cooldownDurationSec);

    /**
     * @brief Evaluates step accumulation and updates inactivity state timers.
     * @param totalSteps Current total valid steps accumulated by the step detector.
     * @param currentActivityState Current ML activity state code.
     * @param nowMs Current system timestamp in ms.
     */
    void processStepActivity(uint32_t totalSteps, uint8_t currentActivityState, uint32_t nowMs);

    /**
     * @brief Manually resets inactivity and cooldown timers.
     */
    void resetTimer();

    /**
     * @brief Registers the callback to execute when an inactivity alert is triggered.
     */
    void setInactivityAlertCallback(InactivityAlertCallback cb) { _onInactivityAlert = cb; }

    // Getters
    bool isEnabled() const { return _enabled; }
    uint16_t getThresholdSec() const { return _inactivityThresholdSec; }
    uint16_t getCooldownSec() const { return _cooldownDurationSec; }
    bool isAlertTriggered() const { return _alertTriggered; }
    uint32_t getLastStepTimestampMs() const { return _lastStepTimestampMs; }
    uint32_t getLastAlertTimestampMs() const { return _lastAlertTimestampMs; }

private:
    bool _enabled;
    uint16_t _inactivityThresholdSec;
    uint16_t _cooldownDurationSec;

    uint32_t _lastStepTimestampMs;
    uint32_t _lastAlertTimestampMs;
    uint32_t _lastObservedTotalSteps;
    uint32_t _stepCountAtAlert;
    bool _alertTriggered;

    InactivityAlertCallback _onInactivityAlert;
    Preferences _prefs;
};

