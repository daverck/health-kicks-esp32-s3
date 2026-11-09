#include "led_driver.h"

LedDriver::LedDriver()
    : _pin(PIN_LED_BLUE),
      _bleState(LedBleState::ADVERTISING),
      _isCalibrating(false),
      _isStudioRecording(false),
      _isOtaUpdating(false),
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

void LedDriver::setOtaUpdating(bool active) {
    if (_isOtaUpdating != active) {
        _isOtaUpdating = active;
        _stateTimestampMs = millis();
    }
}

void LedDriver::updateOtaPattern(uint32_t nowMs) {
    // Fast blinking during OTA flashing (100ms ON / 100ms OFF = 5 Hz)
    uint32_t phase = (nowMs - _stateTimestampMs) % 200;
    setPinState(phase < 100);
}

void LedDriver::updateAdvertisingPattern(uint32_t nowMs) {
    // Periodic pairing blink: 250ms ON / 250ms OFF (2 Hz cycle)
    uint32_t phase = (nowMs - _stateTimestampMs) % 500;
    setPinState(phase < 250);
}

void LedDriver::updateConnectedPattern(uint32_t nowMs) {
    // Solid ON for 2000 ms to confirm connection, then OFF to conserve power
    setPinState((nowMs - _stateTimestampMs) < 2000);
}

void LedDriver::updateDisconnectedPattern(uint32_t nowMs) {
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
}

void LedDriver::update(uint32_t nowMs) {
    // 0. Highest priority: Fast blinking during OTA flashing
    if (_isOtaUpdating) {
        updateOtaPattern(nowMs);
        return;
    }

    // 1. High-priority override: Solid ON during IMU calibration or Studio capture
    if (_isCalibrating || _isStudioRecording) {
        setPinState(true);
        return;
    }

    // 2. BLE Connection state patterns
    switch (_bleState) {
        case LedBleState::ADVERTISING:
            updateAdvertisingPattern(nowMs);
            break;

        case LedBleState::CONNECTED:
            updateConnectedPattern(nowMs);
            break;

        case LedBleState::DISCONNECTED:
            updateDisconnectedPattern(nowMs);
            break;

        case LedBleState::IDLE_OFF:
        default:
            setPinState(false);
            break;
    }
}
