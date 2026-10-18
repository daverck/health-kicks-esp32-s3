#include "offline_fall_storage.h"
#include "ble/ble_server.h"
#include <Arduino.h>
#include <Preferences.h>

static Preferences s_prefs;
static const char* NVS_NAMESPACE = "hk_falls";

OfflineFallStorage::OfflineFallStorage()
    : _count(0), _head(0), _initialized(false) {}

OfflineFallStorage::~OfflineFallStorage() {
    if (_initialized) {
        s_prefs.end();
    }
}

bool OfflineFallStorage::begin() {
    if (!s_prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[NVS-FALL] Failed to initialize NVS namespace!");
        return false;
    }
    _count = s_prefs.getUChar("count", 0);
    _head = s_prefs.getUChar("head", 0);
    if (_count > MAX_FALLS) _count = MAX_FALLS;
    if (_head >= MAX_FALLS) _head = 0;
    _initialized = true;

    if (_count > 0) {
        Serial.printf("[NVS-FALL] Initialized: %u pending offline fall alert(s) in flash memory.\n", _count);
    }
    return true;
}

bool OfflineFallStorage::saveFall(uint8_t stateCode, uint8_t confidence, uint32_t timestampSec) {
    if (!_initialized && !begin()) {
        return false;
    }

    OfflineFallRecord record;
    record.stateCode = stateCode;
    record.confidence = confidence;
    record.timestampSec = timestampSec;

    char key[16];
    snprintf(key, sizeof(key), "fall_%u", _head);

    size_t written = s_prefs.putBytes(key, &record, sizeof(record));
    if (written != sizeof(record)) {
        Serial.printf("[NVS-FALL] Error persisting fall record to key %s!\n", key);
        return false;
    }

    _head = (_head + 1) % MAX_FALLS;
    if (_count < MAX_FALLS) {
        _count++;
    }

    s_prefs.putUChar("head", _head);
    s_prefs.putUChar("count", _count);

    Serial.printf("[NVS-FALL] Successfully saved offline fall to flash: state=0x%02X, conf=%u%%, ts=%u (Total queued: %u/%u)\n",
                  stateCode, confidence, timestampSec, _count, MAX_FALLS);
    return true;
}

bool OfflineFallStorage::hasPendingFalls() const {
    return _count > 0;
}

uint8_t OfflineFallStorage::getPendingCount() const {
    return _count;
}

uint8_t OfflineFallStorage::flushToBle(HealthKicksBleServer& bleServer) {
    if (!_initialized || _count == 0 || !bleServer.isConnected()) {
        return 0;
    }

    Serial.printf("[NVS-FALL] Flushing %u buffered offline fall(s) to connected mobile client...\n", _count);

    uint8_t startIdx = (_count == MAX_FALLS) ? _head : 0;
    uint8_t flushed = 0;

    for (uint8_t i = 0; i < _count; ++i) {
        uint8_t slot = (startIdx + i) % MAX_FALLS;
        char key[16];
        snprintf(key, sizeof(key), "fall_%u", slot);

        OfflineFallRecord record;
        if (s_prefs.getBytes(key, &record, sizeof(record)) == sizeof(record)) {
            bleServer.notifyActivity(
                record.stateCode,
                record.confidence,
                record.timestampSec,
                DETECTION_FLAG_CRITICAL_FALL | DETECTION_FLAG_OFFLINE_BUFFERED
            );
            flushed++;
            delay(20);
        }
    }

    clear();
    Serial.printf("[NVS-FALL] Successfully flushed %u offline fall alert(s) to BLE.\n", flushed);
    return flushed;
}

void OfflineFallStorage::clear() {
    if (!_initialized) return;
    s_prefs.clear();
    _count = 0;
    _head = 0;
    s_prefs.putUChar("count", 0);
    s_prefs.putUChar("head", 0);
}

