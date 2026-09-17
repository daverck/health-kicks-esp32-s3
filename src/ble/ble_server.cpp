#include "ble_server.h"

// Callbacks NimBLE pour le serveur
class ServerCallbacks : public NimBLEServerCallbacks {
public:
    ServerCallbacks(HealthKicksBleServer* parent) : _parent(parent) {}
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        Serial.printf("[BLE] Client connecté (conn_handle: %d). Négociation MTU en cours...\n", desc->conn_handle);
        _parent->onConnect(pServer, desc);
    }
    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        Serial.println("[BLE] Client déconnecté. Redémarrage de l'advertising...");
        _parent->onDisconnect(pServer);
        NimBLEDevice::getAdvertising()->start();
    }
    void onDisconnect(NimBLEServer* pServer) override {
        Serial.println("[BLE] Client déconnecté. Redémarrage de l'advertising...");
        _parent->onDisconnect(pServer);
        NimBLEDevice::getAdvertising()->start();
    }
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
        _parent->onMtuChange(MTU, desc);
    }
private:
    HealthKicksBleServer* _parent;
};

// Callbacks pour l'écriture de commande haptique (0003)
class HapticCharCallbacks : public NimBLECharacteristicCallbacks {
public:
    HapticCharCallbacks(HealthKicksBleServer* parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string val = pCharacteristic->getValue();
        _parent->handleHapticWrite(reinterpret_cast<const uint8_t*>(val.data()), val.length());
    }
private:
    HealthKicksBleServer* _parent;
};

// Callbacks pour l'écriture de commande Studio Control (0004)
class StudioControlCallbacks : public NimBLECharacteristicCallbacks {
public:
    StudioControlCallbacks(HealthKicksBleServer* parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string val = pCharacteristic->getValue();
        _parent->handleStudioControlWrite(reinterpret_cast<const uint8_t*>(val.data()), val.length());
    }
private:
    HealthKicksBleServer* _parent;
};

HealthKicksBleServer::HealthKicksBleServer()
    : _pServer(nullptr),
      _pService(nullptr),
      _pCharActivity(nullptr),
      _pCharHaptic(nullptr),
      _pCharStudioControl(nullptr),
      _pCharStudioBurst(nullptr),
      _deviceConnected(false),
      _negotiatedMtu(23),
      _lastActivityTime(0) {}

void HealthKicksBleServer::begin(const char* deviceName) {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setMTU(BLE_PREFERRED_MTU);

    _pServer = NimBLEDevice::createServer();
    _pServer->setCallbacks(new ServerCallbacks(this));

    // Création du service principal Footwear
    _pService = _pServer->createService(HEALTHKICKS_SERVICE_UUID);

    // Caractéristique 1 : Activity Detection (READ, NOTIFY - 7 octets Big-Endian)
    _pCharActivity = _pService->createCharacteristic(
        CHAR_ACTIVITY_DETECTION_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    uint8_t defaultActivity[7] = {STATE_CODE_IDLE, 0, 0, 0, 0, 0, 0};
    _pCharActivity->setValue(defaultActivity, sizeof(defaultActivity));

    // Caractéristique 2 : Haptic Command (WRITE, WRITE_NR - 4 octets Big-Endian)
    _pCharHaptic = _pService->createCharacteristic(
        CHAR_HAPTIC_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    _pCharHaptic->setCallbacks(new HapticCharCallbacks(this));

    // Caractéristique 3 : Studio Control (WRITE, NOTIFY - ASCII UTF-8)
    _pCharStudioControl = _pService->createCharacteristic(
        CHAR_STUDIO_CONTROL_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    _pCharStudioControl->setCallbacks(new StudioControlCallbacks(this));

    // Caractéristique 4 : Studio Data Burst (NOTIFY - Paquets MTU)
    _pCharStudioBurst = _pService->createCharacteristic(
        CHAR_STUDIO_DATA_BURST_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // Démarrage du service
    _pService->start();

    _lastActivityTime = millis();
    log_i("Serveur NimBLE configuré avec service Footwear: %s", HEALTHKICKS_SERVICE_UUID);

    startAdvertising();
}

void HealthKicksBleServer::startAdvertising() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(BLE_DEVICE_NAME);
    pAdvertising->addServiceUUID(HEALTHKICKS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Intervalle 7.5ms pour réactivité
    pAdvertising->setMaxPreferred(0x12); // Intervalle 22.5ms

    pAdvertising->start();
    _lastActivityTime = millis();
    Serial.printf("[BLE] Publicité BLE démarrée (Nom: %s, Service: %s)\n", BLE_DEVICE_NAME, HEALTHKICKS_SERVICE_UUID);
}

void HealthKicksBleServer::stopAdvertising() {
    NimBLEDevice::getAdvertising()->stop();
    Serial.println("[BLE] Publicité BLE arrêtée.");
}

void HealthKicksBleServer::onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    _deviceConnected = true;
    _lastActivityTime = millis();
    Serial.printf("[BLE] Client BLE connecté (Handle: %d). Attente négociation MTU...\n", desc->conn_handle);
}

void HealthKicksBleServer::onDisconnect(NimBLEServer* pServer) {
    _deviceConnected = false;
    _lastActivityTime = millis();
    Serial.println("[BLE] Client BLE déconnecté. Relance de la publicité...");
    startAdvertising();
}

void HealthKicksBleServer::onMtuChange(uint16_t MTU, ble_gap_conn_desc* desc) {
    _negotiatedMtu = MTU;
    Serial.printf("[BLE] MTU négociée mise à jour: %d octets\n", MTU);
}

void HealthKicksBleServer::handleHapticWrite(const uint8_t* data, size_t length) {
    _lastActivityTime = millis();
    if (length >= 4) {
        uint8_t pattern = data[0];
        uint8_t intensity = data[1];
        uint16_t durationMs = (static_cast<uint16_t>(data[2]) << 8) | data[3];

        Serial.printf("[BLE] Commande haptique reçue: Pattern=%u, Intensité=%u/255, Durée=%u ms\n",
                      pattern, intensity, durationMs);

        if (_onHaptic) {
            _onHaptic(pattern, intensity, durationMs);
        }
    } else {
        Serial.printf("[BLE] Commande haptique invalide (taille %u < 4)\n", length);
    }
}

void HealthKicksBleServer::handleStudioControlWrite(const uint8_t* data, size_t length) {
    _lastActivityTime = millis();
    if (length == 0) return;

    String cmd = "";
    for (size_t i = 0; i < length; i++) {
        cmd += static_cast<char>(data[i]);
    }
    cmd.trim();

    log_i("BLE Studio Control Write reçu: \"%s\"", cmd.c_str());

    if (_onStudioCommand) {
        _onStudioCommand(cmd);
    }
}

void HealthKicksBleServer::notifyActivity(uint8_t stateCode, uint8_t confidence, uint32_t timestampSec, uint8_t flags) {
    if (!_deviceConnected || !_pCharActivity) return;

    uint8_t payload[7];
    payload[0] = stateCode;
    payload[1] = confidence;
    payload[2] = (timestampSec >> 24) & 0xFF;
    payload[3] = (timestampSec >> 16) & 0xFF;
    payload[4] = (timestampSec >> 8) & 0xFF;
    payload[5] = timestampSec & 0xFF;
    payload[6] = flags;

    _pCharActivity->setValue(payload, sizeof(payload));
    _pCharActivity->notify();
}

void HealthKicksBleServer::notifyStudioControl(const String& message) {
    if (!_deviceConnected || !_pCharStudioControl) return;

    _pCharStudioControl->setValue(message.c_str());
    _pCharStudioControl->notify();
    log_i("BLE Notification Studio Control émise: \"%s\"", message.c_str());
}

bool HealthKicksBleServer::sendBurstPacket(const uint8_t* data, size_t length) {
    if (!_deviceConnected || !_pCharStudioBurst) return false;

    _pCharStudioBurst->setValue(data, length);
    _pCharStudioBurst->notify();
    return true;
}

