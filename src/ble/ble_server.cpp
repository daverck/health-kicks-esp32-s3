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
    void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
        if (desc->sec_state.encrypted) {
            _parent->setLinkEncrypted(true);
            _parent->setPeerAuthorized(true);
            Serial.printf("[BLE-SEC] Link encrypted & authenticated (Bonded: %d, LESC: %d).\n",
                          desc->sec_state.bonded, desc->sec_state.authenticated);
        } else {
            _parent->setLinkEncrypted(false);
            _parent->setPeerAuthorized(false);
            if (!_parent->isPairingWindowActive()) {
                Serial.println("[BLE-SEC] Authentication failed without encryption while pairing window is CLOSED. Disconnecting unauthorized peer.");
                if (_parent->getServer()) {
                    _parent->getServer()->disconnect(desc->conn_handle);
                }
            } else {
                Serial.println("[BLE-SEC] Pairing window is OPEN: keeping link active for user pairing confirmation (single prompt).");
            }
        }
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

// Callbacks for OTA Control write (0011)
class OtaControlCallbacks : public NimBLECharacteristicCallbacks {
public:
    OtaControlCallbacks(HealthKicksBleServer* parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string val = pCharacteristic->getValue();
        _parent->handleOtaControlWrite(reinterpret_cast<const uint8_t*>(val.data()), val.length());
    }
private:
    HealthKicksBleServer* _parent;
};

// Callbacks for OTA Data write (0012)
class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
public:
    OtaDataCallbacks(HealthKicksBleServer* parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string val = pCharacteristic->getValue();
        _parent->handleOtaDataWrite(reinterpret_cast<const uint8_t*>(val.data()), val.length());
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
      _pOtaService(nullptr),
      _pCharOtaControl(nullptr),
      _pCharOtaData(nullptr),
      _deviceConnected(false),
      _linkEncrypted(false),
      _peerAuthorized(false),
      _rescueWindowUntilMs(0),
      _negotiatedMtu(23),
      _lastActivityTime(0),
      _connHandle(BLE_HS_CONN_HANDLE_NONE),
      _onHaptic(nullptr),
      _onStudioCommand(nullptr),
      _onCalibration(nullptr),
      _onInactivityConfig(nullptr) {}

void HealthKicksBleServer::begin(const char* deviceName) {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setMTU(BLE_PREFERRED_MTU);

    // Configure BLE Security Manager: LE Secure Connections (LESC) + Bonding (Option 1A: Just Works)
    NimBLEDevice::setSecurityAuth(true, false, true); // bonding = true, mitm = false, sc = true
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

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

    // Characteristic 2: Haptic Command (WRITE, WRITE_NR - 4 or 6 bytes Big-Endian)
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

    // Start primary Footwear service
    _pService->start();
    log_i("NimBLE server configured with Footwear service: %s", HEALTHKICKS_SERVICE_UUID);

    // Create OTA Firmware Update Service (0010)
    _pOtaService = _pServer->createService(HEALTHKICKS_OTA_SERVICE_UUID);

    // OTA Characteristic 1: OTA Control (WRITE, NOTIFY - 0011)
    _pCharOtaControl = _pOtaService->createCharacteristic(
        CHAR_OTA_CONTROL_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
    );
    _pCharOtaControl->setCallbacks(new OtaControlCallbacks(this));

    // OTA Characteristic 2: OTA Data (WRITE, WRITE_NR - 0012)
    _pCharOtaData = _pOtaService->createCharacteristic(
        CHAR_OTA_DATA_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    _pCharOtaData->setCallbacks(new OtaDataCallbacks(this));

    // Start OTA service
    _pOtaService->start();
    log_i("NimBLE server configured with OTA service: %s", HEALTHKICKS_OTA_SERVICE_UUID);

    // Initialize OtaBleService logic
    _otaService.begin([this](const uint8_t* data, size_t len) {
        this->notifyOtaControl(data, len);
    });
    _otaService.setStateChangeCallback([this](bool active) {
        if (active) {
            requestFastBurstConnectionParams();
        } else {
            requestNominalConnectionParams();
        }
    });

    // Create standard Bluetooth SIG Device Information Service (0x180A)
    NimBLEService* pDisService = _pServer->createService(NimBLEUUID((uint16_t)0x180A));
    if (pDisService != nullptr) {
        // Firmware Revision String (0x2A26)
        NimBLECharacteristic* pCharFwRev = pDisService->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A26),
            NIMBLE_PROPERTY::READ
        );
        if (pCharFwRev != nullptr) {
            pCharFwRev->setValue(FIRMWARE_VERSION);
        }

        // Software Revision String (0x2A28) - Build Date / Timestamp
        NimBLECharacteristic* pCharSwRev = pDisService->createCharacteristic(
            NimBLEUUID((uint16_t)0x2A28),
            NIMBLE_PROPERTY::READ
        );
        if (pCharSwRev != nullptr) {
            pCharSwRev->setValue(FIRMWARE_BUILD_DATE);
        }

        pDisService->start();
        log_i("NimBLE server configured with Device Information service: 0x180A");
    }

    _lastActivityTime = millis();
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
    NimBLEAddress peerAddr(desc->peer_ota_addr);
    NimBLEAddress peerIdAddr(desc->peer_id_addr);
    bool alreadyBonded = desc->sec_state.bonded || NimBLEDevice::isBonded(peerAddr) || NimBLEDevice::isBonded(peerIdAddr);
    int numBonds = NimBLEDevice::getNumBonds();
    bool pairingAllowed = isPairingWindowActive() || (numBonds == 0);

    if (!alreadyBonded && !pairingAllowed) {
        _peerAuthorized = false;
        _linkEncrypted = false;
        Serial.printf("[BLE-SEC] REJECTED unbonded device %s (Bonds in NVS: %d, Pairing window: CLOSED). Press shoe button (GPIO 14) to pair a new phone.\n",
                      peerAddr.toString().c_str(), numBonds);
        pServer->disconnect(desc->conn_handle);
        return;
    }

    // If pairing window is explicitly open, purge any stale bond for this peer so pairing starts clean
    if (isPairingWindowActive()) {
        if (NimBLEDevice::isBonded(peerAddr)) {
            Serial.printf("[BLE-SEC] Pairing window active: deleting stale bond for %s to ensure clean pairing.\n",
                          peerAddr.toString().c_str());
            NimBLEDevice::deleteBond(peerAddr);
        }
        if (NimBLEDevice::isBonded(peerIdAddr)) {
            Serial.printf("[BLE-SEC] Pairing window active: deleting stale bond for ID %s to ensure clean pairing.\n",
                          peerIdAddr.toString().c_str());
            NimBLEDevice::deleteBond(peerIdAddr);
        }
    }

    _deviceConnected = true;
    _connHandle = desc->conn_handle;
    _peerAuthorized = false;
    _linkEncrypted = desc->sec_state.encrypted;
    _lastActivityTime = millis();
    requestNominalConnectionParams();
    Serial.printf("[BLE-SEC] Client %s connected (Handle: %d, AlreadyBonded: %d, PairingWindow: %d). Starting LESC security...\n",
                  peerAddr.toString().c_str(), desc->conn_handle, alreadyBonded, isPairingWindowActive());
    NimBLEDevice::startSecurity(desc->conn_handle);
}

void HealthKicksBleServer::onDisconnect(NimBLEServer* pServer) {
    _deviceConnected = false;
    _linkEncrypted = false;
    _peerAuthorized = false;
    _connHandle = BLE_HS_CONN_HANDLE_NONE;
    _lastActivityTime = millis();
    if (_otaService.isOtaInProgress()) {
        Serial.println("[BLE] Client disconnected during OTA! Aborting OTA session...");
        _otaService.abort();
    }
    Serial.println("[BLE] BLE client disconnected.");
}

void HealthKicksBleServer::onMtuChange(uint16_t MTU, ble_gap_conn_desc* desc) {
    _negotiatedMtu = MTU;
    Serial.printf("[BLE] Negotiated MTU updated: %d bytes\n", MTU);
}

void HealthKicksBleServer::handleHapticWrite(const uint8_t* data, size_t length) {
    if ((!_linkEncrypted || !_peerAuthorized) && !isPairingWindowActive()) {
        Serial.println("[BLE-SEC] Ignored write from unauthorized/unencrypted BLE peer.");
        return;
    }
    _lastActivityTime = millis();

    Serial.printf("[BLE] Raw haptic command received (len %u): ", (unsigned int)length);
    for (size_t i = 0; i < length; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    // Characteristic 0003 is strictly reserved for direct haptic commands (4 bytes: pattern, intensity, duration_ms)
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
    if ((!_linkEncrypted || !_peerAuthorized) && !isPairingWindowActive()) {
        Serial.println("[BLE-SEC] Ignored write from unauthorized/unencrypted BLE peer.");
        return;
    }
    _lastActivityTime = millis();
    if (length == 0) return;

    // Fixed stack buffer avoiding dynamic heap String allocations and fragmentation
    char cmdBuf[128];
    size_t copyLen = (length < sizeof(cmdBuf) - 1) ? length : (sizeof(cmdBuf) - 1);
    memcpy(cmdBuf, data, copyLen);
    cmdBuf[copyLen] = '\0';

    // In-place trim of trailing control/whitespace characters
    while (copyLen > 0 && (cmdBuf[copyLen - 1] == ' ' || cmdBuf[copyLen - 1] == '\r' || cmdBuf[copyLen - 1] == '\n' || cmdBuf[copyLen - 1] == '\t')) {
        cmdBuf[--copyLen] = '\0';
    }

    // Skip leading whitespace
    const char* pCmd = cmdBuf;
    while (*pCmd == ' ' || *pCmd == '\t') {
        pCmd++;
    }

    if (*pCmd == '\0') return;

    log_i("BLE Device & Studio Control Write received: \"%s\"", pCmd);

    // 1. SET_INACTIVITY <enabled:0|1> <threshold_sec> <cooldown_sec>
    if (strncmp(pCmd, "SET_INACTIVITY", 14) == 0) {
        int enabledInt = 0;
        unsigned int thresh = 0;
        unsigned int cool = 0;
        if (sscanf(pCmd + 14, "%d %u %u", &enabledInt, &thresh, &cool) == 3) {
            bool enabled = (enabledInt != 0);
            Serial.printf("[BLE] Device Control SET_INACTIVITY: enabled=%d, thresh=%u s, cool=%u s\n",
                          enabled, (uint16_t)thresh, (uint16_t)cool);
            if (_onInactivityConfig) {
                _onInactivityConfig(enabled, (uint16_t)thresh, (uint16_t)cool);
            }
            notifyStudioControl("INACTIVITY_CONFIG_OK");
            return;
        }
        notifyStudioControl("INACTIVITY_CONFIG_ERROR invalid_params");
        return;
    }

    // 2. SET_FALL_IMPACT <impact_ms2>
    if (strncmp(pCmd, "SET_FALL_IMPACT", 15) == 0) {
        float impact = 0.0f;
        if (sscanf(pCmd + 15, "%f", &impact) == 1) {
            if (impact >= MIN_FALL_IMPACT_MIN_MS2 && impact <= MIN_FALL_IMPACT_MAX_MS2) {
                Serial.printf("[BLE] Device Control SET_FALL_IMPACT: %.2f m/s2 (%.2f g)\n", impact, impact / 9.80665f);
                if (_onFallImpactConfig) {
                    _onFallImpactConfig(impact);
                }
                char resp[32];
                snprintf(resp, sizeof(resp), "FALL_IMPACT_OK %.1f", impact);
                notifyStudioControl(resp);
                return;
            } else {
                Serial.printf("[BLE] Device Control SET_FALL_IMPACT rejected: %.2f out of range [%.1f, %.1f]\n",
                              impact, MIN_FALL_IMPACT_MIN_MS2, MIN_FALL_IMPACT_MAX_MS2);
                notifyStudioControl("FALL_IMPACT_ERROR out_of_range");
                return;
            }
        }
        notifyStudioControl("FALL_IMPACT_ERROR invalid_params");
        return;
    }

    // 3. SET_HITL <enabled:0|1>
    if (strncmp(pCmd, "SET_HITL", 8) == 0) {
        int val = -1;
        if (sscanf(pCmd + 8, "%d", &val) == 1 && (val == 0 || val == 1)) {
            _hitlEnabled = (val == 1);
            Serial.printf("[BLE] Device Control SET_HITL: %d\n", val);
            if (_onHitlConfig) {
                _onHitlConfig(_hitlEnabled);
            }
            char resp[32];
            snprintf(resp, sizeof(resp), "HITL_OK %d", val);
            notifyStudioControl(resp);
            return;
        }
        notifyStudioControl("HITL_ERROR invalid_params");
        return;
    }

    // 4. CALIBRATE
    if (strcasecmp(pCmd, "CALIBRATE") == 0 || strncasecmp(pCmd, "CALIB", 5) == 0) {
        Serial.println("[BLE] Device Control CALIBRATE command received");
        if (_onCalibration) {
            _onCalibration();
        }
        return;
    }

    // 5. Studio commands (START, FINISH, CANCEL) dispatched via callback
    if (_onStudioCommand) {
        _onStudioCommand(std::string(pCmd));
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

void HealthKicksBleServer::requestFastBurstConnectionParams() {
    if (_pServer && _deviceConnected && _connHandle != BLE_HS_CONN_HANDLE_NONE) {
        Serial.printf("[BLE] Requesting Fast transfer connection interval: 15-20 ms (conn_handle: %d)\n", _connHandle);
        _pServer->updateConnParams(_connHandle, 12, 16, 0, 400);
    }
}

void HealthKicksBleServer::requestNominalConnectionParams() {
    if (_pServer && _deviceConnected && _connHandle != BLE_HS_CONN_HANDLE_NONE) {
        Serial.printf("[BLE] Restoring Nominal connection interval: 100-125 ms (conn_handle: %d)\n", _connHandle);
        _pServer->updateConnParams(_connHandle, 80, 100, 0, 400);
    }
}

void HealthKicksBleServer::handleOtaControlWrite(const uint8_t* data, size_t length) {
    if ((!_linkEncrypted || !_peerAuthorized) && !isPairingWindowActive()) {
        Serial.println("[BLE-SEC] Ignored write from unauthorized/unencrypted BLE peer.");
        return;
    }
    _lastActivityTime = millis();
    _otaService.handleControlWrite(data, length);
}

void HealthKicksBleServer::handleOtaDataWrite(const uint8_t* data, size_t length) {
    if ((!_linkEncrypted || !_peerAuthorized) && !isPairingWindowActive()) {
        Serial.println("[BLE-SEC] Ignored write from unauthorized/unencrypted BLE peer.");
        return;
    }
    _lastActivityTime = millis();
    _otaService.handleDataWrite(data, length);
}

void HealthKicksBleServer::notifyOtaControl(const uint8_t* data, size_t length) {
    if (!_deviceConnected || !_pCharOtaControl) return;

    _pCharOtaControl->setValue(data, length);
    _pCharOtaControl->notify();
}
