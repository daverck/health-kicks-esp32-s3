#include "led_driver.h"

LedDriver::LedDriver()
    : _pin(PIN_LED_BLUE),
      _bleState(LedBleState::ADVERTISING),
      _isCalibrating(false),
      _isStudioRecording(false),
      _stateTimestampMs(0),
      _currentPinState(false) {}

void LedDriver::init(int pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    _currentPinState = false;
    _stateTimestampMs = millis();
}

void LedDriver::setPinState(bool high) {
    if (_currentPinState != high) {
        _currentPinState = high;
        digitalWrite(_pin, high ? HIGH : LOW);
    }
}

void LedDriver::setBleAdvertising() {
    _bleState = LedBleState::ADVERTISING;
    _stateTimestampMs = millis();
}

void LedDriver::setBleConnected() {
    _bleState = LedBleState::CONNECTED;
    _stateTimestampMs = millis();
}

void LedDriver::setBleDisconnected() {
    _bleState = LedBleState::DISCONNECTED;
    _stateTimestampMs = millis();
}

void LedDriver::setCalibrating(bool active) {
    _isCalibrating = active;
}

void LedDriver::setStudioRecording(bool active) {
    _isStudioRecording = active;
}

void LedDriver::update(uint32_t nowMs) {
    // 1. High-priority override: Solid ON during IMU calibration or Studio capture
    if (_isCalibrating || _isStudioRecording) {
        setPinState(true);
        return;
    }

    // 2. BLE Connection state patterns
    switch (_bleState) {
        case LedBleState::ADVERTISING: {
            // Periodic pairing blink: 250ms ON / 250ms OFF (2 Hz cycle)
            uint32_t phase = (nowMs - _stateTimestampMs) % 500;
            setPinState(phase < 250);
            break;
        }

        case LedBleState::CONNECTED: {
            // Solid ON for 2000 ms to confirm connection, then OFF to conserve power
            if (nowMs - _stateTimestampMs < 2000) {
                setPinState(true);
            } else {
                setPinState(false);
            }
            break;
        }

        case LedBleState::DISCONNECTED: {
            // Alert pattern: double flash (100ms ON - 100ms OFF - 100ms ON - 100ms OFF = 400ms)
            uint32_t elapsed = nowMs - _stateTimestampMs;
            if (elapsed < 100) {
                setPinState(true);
            } else if (elapsed < 200) {
                setPinState(false);
            } else if (elapsed < 300) {
                setPinState(true);
            } else if (elapsed < 400) {
                setPinState(false);
            } else {
                // Return to standard advertising blink once double flash is complete
                _bleState = LedBleState::ADVERTISING;
                _stateTimestampMs = nowMs;
            }
            break;
        }

        case LedBleState::IDLE_OFF:
        default:
            setPinState(false);
            break;
    }
}
