#pragma once

#include <Arduino.h>
#include <functional>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"
#include "invariants.h"

typedef std::function<void(const uint8_t* data, size_t len)> OtaNotifyCallback;
typedef std::function<void(bool active)> OtaStateChangeCallback;

/**
 * @brief Over-The-Air BLE firmware update manager using native ESP-IDF OTA API.
 * Contract reference: contracts/ble_gatt_specs.md (Service 0010, Chars 0011 & 0012)
 */
class OtaBleService {
public:
    OtaBleService();

    /**
     * @brief Initializes callback for sending notifications on OTA Control characteristic.
     */
    void begin(OtaNotifyCallback notifyCb);

    /**
     * @brief Handles raw incoming control frames on Characteristic 0011 (OTA Control).
     * @param data Byte buffer containing opcode and payload.
     * @param len Buffer length in bytes.
     */
    void handleControlWrite(const uint8_t* data, size_t len);

    /**
     * @brief Handles raw firmware binary chunks on Characteristic 0012 (OTA Data).
     * @param data Binary chunk slice.
     * @param len Chunk length in bytes.
     */
    void handleDataWrite(const uint8_t* data, size_t len);

    /**
     * @brief Returns true if an OTA flash session is actively underway.
     */
    bool isOtaInProgress() const { return _inProgress; }

    /**
     * @brief Returns the total expected binary size in bytes.
     */
    uint32_t getTotalSize() const { return _totalSize; }

    /**
     * @brief Returns the number of bytes written so far to the inactive flash slot.
     */
    uint32_t getBytesWritten() const { return _bytesWritten; }

    /**
     * @brief Registers callback triggered when OTA begins or finishes/aborts.
     */
    void setStateChangeCallback(OtaStateChangeCallback cb) { _onStateChange = cb; }

    /**
     * @brief Emergency abort of current OTA update.
     */
    void abort();

private:
    bool _inProgress;
    uint32_t _totalSize;
    uint32_t _bytesWritten;
    esp_ota_handle_t _otaHandle;
    const esp_partition_t* _updatePartition;

    OtaNotifyCallback _notifyControl;
    OtaStateChangeCallback _onStateChange;

    mbedtls_sha256_context _sha256Ctx;
    bool _sha256Initialized;

    void notifyError(uint8_t errorCode);
    void notifyStatus(uint8_t statusCode);
};
