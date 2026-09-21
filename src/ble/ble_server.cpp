#include "ble_server.h"

volatile bool g_need_restart_advertising = false;

// NimBLE Callbacks for the server
class ServerCallbacks : public NimBLEServerCallbacks {
public:
    ServerCallbacks(HealthKicksBleServer* parent) : _parent(parent) {}
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        Serial.printf("[BLE] Client connected (conn_handle: %d). Negotiating MTU...\n", desc->conn_handle);
        _parent->onConnect(pServer, desc);
    }
    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        Serial.println("[BLE] Client disconnected. Signalling advertising restart...");
        _parent->onDisconnect(pServer);
        g_need_restart_advertising = true;
    }
    void onDisconnect(NimBLEServer* pServer) override {
        Serial.println("[BLE] Client disconnected. Signalling advertising restart...");
        _parent->onDisconnect(pServer);
        g_need_restart_advertising = true;
    }
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
        _parent->onMtuChange(MTU, desc);
    }
private:
    HealthKicksBleServer* _parent;
};

// Callbacks for Haptic Command write (0003)
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

// Callbacks for Studio Control write (0004)
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
      _pCharStepCounter(nullptr),
      _deviceConnected(false),
      _negotiatedMtu(23),
      _lastActivityTime(0) {}

void HealthKicksBleServer::begin(const char* deviceName) {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setMTU(BLE_PREFERRED_MTU);

    _pServer = NimBLEDevice::createServer();
    _pServer->setCallbacks(new ServerCallbacks(this));

    // Create main Footwear service
    _pService = _pServer->createService(HEALTHKICKS_SERVICE_UUID);

    // Characteristic 1: Activity Detection (READ, NOTIFY - 7 bytes Big-Endian)
    _pCharActivity = _pService->createCharacteristic(
        CHAR_ACTIVITY_DETECTION_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    uint8_t defaultActivity[7] = {STATE_CODE_IDLE, 0, 0, 0, 0, 0, 0};
    _pCharActivity->setValue(defaultActivity, sizeof(defaultActivity));

    // Characteristic 2: Haptic Command (WRITE, WRITE_NR - 4 bytes Big-Endian)
    _pCharHaptic = _pService->createCharacteristic(
        CHAR_HAPTIC_COMMAND_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    _pCharHaptic->setCallbacks(new HapticCharCallbacks(this));

    // Characteristic 3: Studio Control (WRITE, NOTIFY - ASCII UTF-8)
    _pCharStudioControl = _pService->createCharacteristic(
        CHAR_STUDIO_CONTROL_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    _pCharStudioControl->setCallbacks(new StudioControlCallbacks(this));

    // Characteristic 4: Studio Data Burst (NOTIFY - MTU Packets)
    _pCharStudioBurst = _pService->createCharacteristic(
        CHAR_STUDIO_DATA_BURST_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // Characteristic 5: Step Counter / Pedometer (READ, NOTIFY - 13 bytes Big-Endian)
    _pCharStepCounter = _pService->createCharacteristic(
        CHAR_STEP_COUNTER_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    StepCounterPayload defaultSteps = {0, 0, 0, 0, 0, 0};
    _pCharStepCounter->setValue(reinterpret_cast<const uint8_t*>(&defaultSteps), sizeof(defaultSteps));

    // Start service
    _pService->start();

    _lastActivityTime = millis();
    log_i("NimBLE server configured with Footwear service: %s", HEALTHKICKS_SERVICE_UUID);

    startAdvertising();
}

void HealthKicksBleServer::startAdvertising() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(BLE_DEVICE_NAME);
    pAdvertising->addServiceUUID(HEALTHKICKS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // 7.5ms interval for responsiveness
    pAdvertising->setMaxPreferred(0x12); // 22.5ms interval

    pAdvertising->start();
    _lastActivityTime = millis();
    Serial.printf("[BLE] BLE advertising started (Name: %s, Service: %s)\n", BLE_DEVICE_NAME, HEALTHKICKS_SERVICE_UUID);
}

void HealthKicksBleServer::stopAdvertising() {
    NimBLEDevice::getAdvertising()->stop();
    Serial.println("[BLE] BLE advertising stopped.");
}

void HealthKicksBleServer::onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    _deviceConnected = true;
    _lastActivityTime = millis();
    Serial.printf("[BLE] BLE client connected (Handle: %d). Waiting for MTU negotiation...\n", desc->conn_handle);
}

void HealthKicksBleServer::onDisconnect(NimBLEServer* pServer) {
    _deviceConnected = false;
    _lastActivityTime = millis();
    Serial.println("[BLE] BLE client disconnected.");
}

void HealthKicksBleServer::onMtuChange(uint16_t MTU, ble_gap_conn_desc* desc) {
    _negotiatedMtu = MTU;
    Serial.printf("[BLE] Negotiated MTU updated: %d bytes\n", MTU);
}

void HealthKicksBleServer::handleHapticWrite(const uint8_t* data, size_t length) {
    _lastActivityTime = millis();

    Serial.printf("[BLE] Raw haptic command received (len %u): ", (unsigned int)length);
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    // Check for Zero-Calibration command (Opcode 0x05)
    if (length >= 1 && data[0] == CMD_TRIGGER_CALIBRATION) {
        Serial.println("[BLE] Zero-calibration command received (Opcode 0x05)");
        if (_onCalibration) {
            _onCalibration();
        }
        return;
    }

    if (length >= 4) {
        uint8_t pattern = data[0];
        uint8_t intensity = data[1];
        uint16_t duration_ms = ((uint16_t)data[2] << 8) | (uint16_t)data[3];
        if (duration_ms == 0) duration_ms = 400; // Safety fallback

        Serial.printf("[BLE] Haptic command received: Pattern=%u, Intensity=%u/255, Duration=%u ms\n",
                      pattern, intensity, duration_ms);

        if (_onHaptic) {
            _onHaptic(pattern, intensity, duration_ms);
        } else {
            Serial.println("[BLE] WARNING: No haptic callback assigned (_onHaptic is null)!");
        }
    } else {
        Serial.printf("[BLE] Invalid haptic command (len %u < 4)\n", (unsigned int)length);
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

    log_i("BLE Studio Control Write received: \"%s\"", cmd.c_str());

    // Check for ASCII CALIBRATE command
    if (cmd.equalsIgnoreCase("CALIBRATE") || cmd.startsWith("CALIB")) {
        Serial.println("[BLE] Studio Control CALIBRATE command received");
        if (_onCalibration) {
            _onCalibration();
        }
        return;
    }

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

void HealthKicksBleServer::notifyStepCounter(const StepCounterPayload& payload) {
    if (!_deviceConnected || !_pCharStepCounter) return;

    _pCharStepCounter->setValue(reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
    _pCharStepCounter->notify();
}

void HealthKicksBleServer::notifyStudioControl(const std::string& message) {
    if (!_deviceConnected || !_pCharStudioControl) return;

    _pCharStudioControl->setValue(reinterpret_cast<const uint8_t*>(message.data()), message.length());
    _pCharStudioControl->notify();
    Serial.printf("[BLE] Studio Control notification sent: \"%s\"\n", message.c_str());
}

void HealthKicksBleServer::notifyStudioControl(const String& message) {
    notifyStudioControl(std::string(message.c_str()));
}

void HealthKicksBleServer::notifyStudioControl(const char* message) {
    if (!message) return;
    notifyStudioControl(std::string(message));
}

bool HealthKicksBleServer::sendBurstPacket(const uint8_t* data, size_t length) {
    if (!_deviceConnected || !_pCharStudioBurst) return false;

    _pCharStudioBurst->setValue(data, length);
    _pCharStudioBurst->notify();
    return true;
}

