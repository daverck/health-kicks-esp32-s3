#pragma once

#include <Arduino.h>
#include "config.h"

/**
 * @file led_driver.h
 * @brief Non-blocking status LED driver for BLE, IMU Calibration, and Studio Capture.
 */

enum class LedBleState {
    IDLE_OFF,
    ADVERTISING,     // Periodic blinking waiting for BLE client
    CONNECTED,       // Solid ON for 2s confirmation, then OFF
    DISCONNECTED     // Double flash warning, then automatically switches to ADVERTISING
};

class LedDriver {
public:
    LedDriver();

    /**
     * @brief Initializes GPIO pin as OUTPUT and sets initial state to LOW.
     * @param pin GPIO pin connected to the LED anode (default: PIN_LED_BLUE / GPIO 13).
     */
    void init(int pin = PIN_LED_BLUE);
    void begin(int pin = PIN_LED_BLUE) { init(pin); }

    /**
     * @brief Sets BLE connection state to ADVERTISING (starts 2 Hz regular blink).
     */
    void setBleAdvertising();

    /**
     * @brief Sets BLE connection state to CONNECTED (turns solid ON for 2s, then OFF).
     */
    void setBleConnected();

    /**
     * @brief Sets BLE connection state to DISCONNECTED (triggers 2 quick alert pulses, then returns to ADVERTISING).
     */
    void setBleDisconnected();

    /**
     * @brief Sets manual or automatic calibration status.
     * @param active When true, LED stays solid ON until set to false.
     */
    void setCalibrating(bool active);

    /**
     * @brief Sets Studio high-speed capture status.
     * @param active When true, LED stays solid ON until set to false.
     */
    void setStudioRecording(bool active);

    /**
     * @brief Sets Over-The-Air firmware flashing status.
     * @param active When true, LED blinks rapidly (5 Hz) until set to false.
     */
    void setOtaUpdating(bool active);

    /**
     * @brief Non-blocking state machine update called at each loop iteration.
     * @param nowMs Current timestamp from millis().
     */
    void update(uint32_t nowMs);

    bool isCalibrating() const { return _isCalibrating; }
    bool isStudioRecording() const { return _isStudioRecording; }
    bool isOtaUpdating() const { return _isOtaUpdating; }
    LedBleState getBleState() const { return _bleState; }

private:
    int _pin;
    LedBleState _bleState;
    bool _isCalibrating;
    bool _isStudioRecording;
    bool _isOtaUpdating;

    uint32_t _stateTimestampMs;
    bool _currentPinState;

    void setPinState(bool high);
};
