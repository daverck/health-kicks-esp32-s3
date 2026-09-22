#include "inactivity_monitor.h"

InactivityMonitor::InactivityMonitor()
    : _enabled(true),
      _inactivityThresholdSec(DEFAULT_INACTIVITY_THRESHOLD_SEC),
      _cooldownDurationSec(DEFAULT_COOLDOWN_DURATION_SEC),
      _lastStepTimestampMs(0),
      _lastAlertTimestampMs(0),
      _lastObservedTotalSteps(0),
      _stepCountAtAlert(0),
      _alertTriggered(false),
      _onInactivityAlert(nullptr) {}

void InactivityMonitor::begin() {
    _prefs.begin("hk_inact", false);
    _enabled = _prefs.getBool("enabled", true);
    _inactivityThresholdSec = _prefs.getUShort("thresh_s", DEFAULT_INACTIVITY_THRESHOLD_SEC);
    _cooldownDurationSec = _prefs.getUShort("cool_s", DEFAULT_COOLDOWN_DURATION_SEC);

    _lastStepTimestampMs = millis();
    _lastAlertTimestampMs = 0;
    _alertTriggered = false;
    _lastObservedTotalSteps = 0;
    _stepCountAtAlert = 0;

    Serial.printf("[INACTIVITY] Monitor initialized: enabled=%d, thresh=%u s (%u min), cool=%u s (%u min)\n",
                  _enabled, _inactivityThresholdSec, _inactivityThresholdSec / 60,
                  _cooldownDurationSec, _cooldownDurationSec / 60);
}

void InactivityMonitor::configure(bool enabled, uint16_t inactivityThresholdSec, uint16_t cooldownDurationSec) {
    _enabled = enabled;
    _inactivityThresholdSec = inactivityThresholdSec;
    _cooldownDurationSec = cooldownDurationSec;

    _prefs.putBool("enabled", _enabled);
    _prefs.putUShort("thresh_s", _inactivityThresholdSec);
    _prefs.putUShort("cool_s", _cooldownDurationSec);

    Serial.printf("[INACTIVITY] Configuration updated & saved to NVS: enabled=%d, thresh=%u s, cool=%u s\n",
                  _enabled, _inactivityThresholdSec, _cooldownDurationSec);
}

void InactivityMonitor::processStepActivity(uint32_t totalSteps, uint8_t currentActivityState, uint32_t nowMs) {
    if (totalSteps > _lastObservedTotalSteps) {
        // Step detected: update last step timestamp
        _lastStepTimestampMs = nowMs;

        // Check if walking resumed with at least RESUME_STEP_THRESHOLD steps
        if (_alertTriggered && (totalSteps - _stepCountAtAlert >= RESUME_STEP_THRESHOLD)) {
            _alertTriggered = false;
            Serial.println("[INACTIVITY] Walking resumed (>=10 steps). Inactivity timer reset.");
        }
        _lastObservedTotalSteps = totalSteps;
    }

    if (!_enabled) {
        return;
    }

    if (!_alertTriggered) {
        if ((nowMs - _lastStepTimestampMs) >= ((uint32_t)_inactivityThresholdSec * 1000)) {
            _alertTriggered = true;
            _lastAlertTimestampMs = nowMs;
            _stepCountAtAlert = totalSteps;

            Serial.println("[INACTIVITY] Alert triggered! Continuous inactivity threshold exceeded.");
            if (_onInactivityAlert) {
                _onInactivityAlert(nowMs);
            }
        }
    } else {
        if (_cooldownDurationSec > 0 && (nowMs - _lastAlertTimestampMs) >= ((uint32_t)_cooldownDurationSec * 1000)) {
            _lastAlertTimestampMs = nowMs;

            Serial.println("[INACTIVITY] Cooldown repeat alert triggered.");
            if (_onInactivityAlert) {
                _onInactivityAlert(nowMs);
            }
        }
    }
}

void InactivityMonitor::resetTimer() {
    _lastStepTimestampMs = millis();
    _alertTriggered = false;
}

