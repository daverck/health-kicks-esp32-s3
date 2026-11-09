#include "offline_fall_storage.h"
#include "ble/ble_server.h"
#include <Arduino.h>
#include <Preferences.h>

static Preferences s_prefs;
static const char* NVS_NAMESPACE = "hk_falls";

OfflineFallStorage::OfflineFallStorage()
    : _bufferIndex(MAX_FALLS), _initialized(false) {}

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
    uint8_t savedCount = s_prefs.getUChar("count", 0);
    uint8_t savedHead = s_prefs.getUChar("head", 0);
    _bufferIndex.restore(savedHead, savedCount);
    _initialized = true;

    if (!_bufferIndex.isEmpty()) {
        Serial.printf("[NVS-FALL] Initialized: %u pending offline fall alert(s) in flash memory.\n", _bufferIndex.count());
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

    uint8_t writeSlot = _bufferIndex.advanceWrite();
    char key[16];
    snprintf(key, sizeof(key), "fall_%u", writeSlot);

    size_t written = s_prefs.putBytes(key, &record, sizeof(record));
    if (written != sizeof(record)) {
        Serial.printf("[NVS-FALL] Error persisting fall record to key %s!\n", key);
        return false;
    }

    s_prefs.putUChar("head", _bufferIndex.head());
    s_prefs.putUChar("count", _bufferIndex.count());

    Serial.printf("[NVS-FALL] Successfully saved offline fall to flash: state=0x%02X, conf=%u%%, ts=%u (Total queued: %u/%u)\n",
                  stateCode, confidence, timestampSec, _bufferIndex.count(), MAX_FALLS);
    return true;
}

bool OfflineFallStorage::hasPendingFalls() const {
    return !_bufferIndex.isEmpty();
}

uint8_t OfflineFallStorage::getPendingCount() const {
    return _bufferIndex.count();
}

uint8_t OfflineFallStorage::flushToBle(HealthKicksBleServer& bleServer) {
    if (!_initialized || _bufferIndex.isEmpty() || !bleServer.isConnected()) {
        return 0;
    }

    uint8_t pendingCount = _bufferIndex.count();
    Serial.printf("[NVS-FALL] Flushing %u buffered offline fall(s) to connected mobile client...\n", pendingCount);

    uint8_t flushed = 0;

    for (uint8_t i = 0; i < pendingCount; ++i) {
        uint8_t slot = _bufferIndex.getChronologicalSlot(i);
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
    _bufferIndex.reset();
    s_prefs.putUChar("count", 0);
    s_prefs.putUChar("head", 0);
}
