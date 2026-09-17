#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"
#include "invariants.h"

typedef std::function<void(uint8_t pattern, uint8_t intensity, uint16_t durationMs)> HapticCallback;
typedef std::function<void(const String& command)> StudioCommandCallback;

class HealthKicksBleServer {
public:
    HealthKicksBleServer();

    /**
     * @brief Initialise la pile NimBLE, le service Footwear et les 4 caractéristiques.
     */
    void begin(const char* deviceName = BLE_DEVICE_NAME);

    /**
     * @brief Lance la publicité BLE (Advertising).
     */
    void startAdvertising();

    /**
     * @brief Arrête la publicité BLE.
     */
    void stopAdvertising();

    /**
     * @brief Notifie un changement d'état d'activité (Caractéristique 0002, 7 octets Big-Endian).
     */
    void notifyActivity(uint8_t stateCode, uint8_t confidence, uint32_t timestampSec, uint8_t flags = 0);

    /**
     * @brief Notifie un message textuel de session Studio (Caractéristique 0004, ASCII).
     */
    void notifyStudioControl(const String& message);

    /**
     * @brief Émet un paquet binaire haute vitesse sur Studio Data Burst (Caractéristique 0005).
     */
    bool sendBurstPacket(const uint8_t* data, size_t length);

    // Callbacks d'événements
    void setHapticCallback(HapticCallback cb) { _onHaptic = cb; }
    void setStudioCommandCallback(StudioCommandCallback cb) { _onStudioCommand = cb; }

    bool isConnected() const { return _deviceConnected; }
    uint16_t getNegotiatedMtu() const { return _negotiatedMtu; }
    uint32_t getLastActivityTime() const { return _lastActivityTime; }

    // Méthodes internes appelées par les callbacks NimBLE
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

    bool _deviceConnected;
    uint16_t _negotiatedMtu;
    uint32_t _lastActivityTime;

    HapticCallback _onHaptic;
    StudioCommandCallback _onStudioCommand;
};
