#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"
#include "invariants.h"

typedef std::function<void(uint8_t pattern, uint8_t intensity, uint16_t durationMs)> HapticCallback;
typedef std::function<void(const String& command)> StudioCommandCallback;
typedef std::function<void()> CalibrationTriggerCallback;
typedef std::function<void(bool enabled, uint16_t thresholdSec, uint16_t cooldownSec)> InactivityConfigCallback;

// Asynchronous BLE advertising restart flag (handled in loop() to avoid radio deadlocks)
extern volatile bool g_need_restart_advertising;

class HealthKicksBleServer {
public:
    HealthKicksBleServer();

    /**
     * @brief Initializes NimBLE stack, Footwear service and the 5 characteristics.
     */
    void begin(const char* deviceName = BLE_DEVICE_NAME);

    /**
     * @brief Starts BLE advertising.
     */
    void startAdvertising();

    /**
     * @brief Stops BLE advertising.
     */
    void stopAdvertising();

    /**
     * @brief Notifies an activity state change (Characteristic 0002, 7 bytes Big-Endian).
     */
    void notifyActivity(uint8_t stateCode, uint8_t confidence, uint32_t timestampSec, uint8_t flags = 0);

    /**
     * @brief Notifies step counter data (Characteristic 0006, 13 bytes Big-Endian).
     */
    void notifyStepCounter(const StepCounterPayload& payload);

    /**
     * @brief Notifies a text message for Studio session (Characteristic 0004, ASCII).
     */
    void notifyStudioControl(const std::string& message);
    void notifyStudioControl(const String& message);
    void notifyStudioControl(const char* message);

    /**
     * @brief Sends a high-speed binary packet over Studio Data Burst (Characteristic 0005).
     */
    bool sendBurstPacket(const uint8_t* data, size_t length);

    // Event callbacks
    void setHapticCallback(HapticCallback cb) { _onHaptic = cb; }
    void setStudioCommandCallback(StudioCommandCallback cb) { _onStudioCommand = cb; }
    void setCalibrationCallback(CalibrationTriggerCallback cb) { _onCalibration = cb; }
    void setInactivityConfigCallback(InactivityConfigCallback cb) { _onInactivityConfig = cb; }

    bool isConnected() const { return _deviceConnected; }
    uint16_t getNegotiatedMtu() const { return _negotiatedMtu; }
    uint32_t getLastActivityTime() const { return _lastActivityTime; }

    // Internal methods invoked by NimBLE callbacks
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc);
    void onDisconnect(NimBLEServer* pServer);
    void onMtuChange(uint16_t MTU, ble_gap_conn_desc* desc);
    void handleHapticWrite(const uint8_t* data, size_t length);
    void handleStudioControlWrite(const uint8_t* data, size_t length);

private:
    NimBLEServer* _pServer;
    NimBLEService* _pService;
    NimBLECharacteristic* _pCharActivity;
    NimBLECharacteristic* _pCharHaptic;
    NimBLECharacteristic* _pCharStudioControl;
    NimBLECharacteristic* _pCharStudioBurst;
    NimBLECharacteristic* _pCharStepCounter;

    bool _deviceConnected;
    uint16_t _negotiatedMtu;
    uint32_t _lastActivityTime;

    HapticCallback _onHaptic;
    StudioCommandCallback _onStudioCommand;
    CalibrationTriggerCallback _onCalibration;
    InactivityConfigCallback _onInactivityConfig;
};
