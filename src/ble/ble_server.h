#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"
#include "invariants.h"
#include "ota_ble_service.h"

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

    /**
     * @brief Requests dynamic BLE connection parameters update (minInterval/maxInterval in 1.25ms units).
     * Nominal mode: 100-125ms (min=80, max=100, latency=0, timeout=400 -> 4s).
     * Fast Burst mode: 15-20ms (min=12, max=16, latency=0, timeout=400 -> 4s).
     */
    void requestFastBurstConnectionParams();
    void requestNominalConnectionParams();

    // Event callbacks
    void setHapticCallback(HapticCallback cb) { _onHaptic = cb; }
    void setStudioCommandCallback(StudioCommandCallback cb) { _onStudioCommand = cb; }
    void setCalibrationCallback(CalibrationTriggerCallback cb) { _onCalibration = cb; }
    void setInactivityConfigCallback(InactivityConfigCallback cb) { _onInactivityConfig = cb; }

    bool isConnected() const { return _deviceConnected; }
    NimBLEServer* getServer() const { return _pServer; }
    bool isLinkEncrypted() const { return _linkEncrypted; }
    void setLinkEncrypted(bool enc) { _linkEncrypted = enc; }
    bool isPeerAuthorized() const { return _peerAuthorized; }
    void setPeerAuthorized(bool auth) { _peerAuthorized = auth; }

    /**
     * @brief Opens a 60-second pairing window triggered by a short press on the physical shoe button (GPIO 14).
     * Only while this window is active (or when 0 devices are bonded in NVS) can a new phone bond or replace a stale bond.
     */
    void openPairingWindow(uint32_t durationMs = 60000) {
        _rescueWindowUntilMs = millis() + durationMs;
        Serial.printf("[BLE-SEC] Physical Button Pairing Window OPENED for %u ms.\n", durationMs);
    }
    void openRescueWindow(uint32_t durationMs = 60000) { openPairingWindow(durationMs); }
    bool isPairingWindowActive() const {
        return _rescueWindowUntilMs != 0 && (int32_t)(_rescueWindowUntilMs - millis()) > 0;
    }
    bool isRescueWindowActive() const { return isPairingWindowActive(); }
    uint16_t getNegotiatedMtu() const { return _negotiatedMtu; }
    uint32_t getLastActivityTime() const { return _lastActivityTime; }

    // Internal methods invoked by NimBLE callbacks
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc);
    void onDisconnect(NimBLEServer* pServer);
    void onMtuChange(uint16_t MTU, ble_gap_conn_desc* desc);
    void handleHapticWrite(const uint8_t* data, size_t length);
    void handleStudioControlWrite(const uint8_t* data, size_t length);
    void handleOtaControlWrite(const uint8_t* data, size_t length);
    void handleOtaDataWrite(const uint8_t* data, size_t length);
    void notifyOtaControl(const uint8_t* data, size_t length);

    OtaBleService& getOtaService() { return _otaService; }

private:
    NimBLEServer* _pServer;
    NimBLEService* _pService;
    NimBLECharacteristic* _pCharActivity;
    NimBLECharacteristic* _pCharHaptic;
    NimBLECharacteristic* _pCharStudioControl;
    NimBLECharacteristic* _pCharStudioBurst;
    NimBLECharacteristic* _pCharStepCounter;

    NimBLEService* _pOtaService;
    NimBLECharacteristic* _pCharOtaControl;
    NimBLECharacteristic* _pCharOtaData;
    OtaBleService _otaService;

    bool _deviceConnected;
    bool _linkEncrypted;
    bool _peerAuthorized;
    uint32_t _rescueWindowUntilMs;
    uint16_t _negotiatedMtu;
    uint32_t _lastActivityTime;
    uint16_t _connHandle;

    HapticCallback _onHaptic;
    StudioCommandCallback _onStudioCommand;
    CalibrationTriggerCallback _onCalibration;
    InactivityConfigCallback _onInactivityConfig;
};
