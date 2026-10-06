#include "ota_ble_service.h"

OtaBleService::OtaBleService()
    : _inProgress(false),
      _totalSize(0),
      _bytesWritten(0),
      _otaHandle(0),
      _updatePartition(nullptr),
      _notifyControl(nullptr),
      _onStateChange(nullptr),
      _sha256Initialized(false) {}

void OtaBleService::begin(OtaNotifyCallback notifyCb) {
    _notifyControl = notifyCb;
    _inProgress = false;
    _bytesWritten = 0;
    _totalSize = 0;
    _otaHandle = 0;
    _updatePartition = nullptr;
    _sha256Initialized = false;

    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running) {
        Serial.printf("[OTA] Running partition: %s (subtype: 0x%02X, offset: 0x%08X, size: %u bytes)\n",
                      running->label, running->subtype, running->address, running->size);
    }
}

void OtaBleService::notifyStatus(uint8_t statusCode) {
    if (_notifyControl) {
        uint8_t payload[1] = {statusCode};
        _notifyControl(payload, 1);
    }
}

void OtaBleService::notifyError(uint8_t errorCode) {
    if (_notifyControl) {
        uint8_t payload[2] = {OTA_RESP_ERROR, errorCode};
        _notifyControl(payload, 2);
    }
    Serial.printf("[OTA] Sent error notification 0xFF (code: 0x%02X)\n", errorCode);
}

void OtaBleService::handleControlWrite(const uint8_t* data, size_t len) {
    if (len == 0 || !data) return;

    uint8_t cmd = data[0];

    // Command 0x01: OTA_BEGIN [0x01, size_b3, size_b2, size_b1, size_b0]
    if (cmd == OTA_CMD_BEGIN) {
        if (len < 5) {
            Serial.printf("[OTA] Error: OTA_BEGIN packet too short (%u bytes < 5)\n", (unsigned int)len);
            notifyError(OTA_ERR_BEGIN_FAILED);
            return;
        }

        if (_inProgress) {
            Serial.println("[OTA] Warning: OTA already in progress, aborting previous session.");
            abort();
        }

        _totalSize = ((uint32_t)data[1] << 24) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 8)  |
                     (uint32_t)data[4];

        Serial.printf("[OTA] OTA_BEGIN received. Declared binary size: %u bytes (%.2f KB)\n",
                      _totalSize, _totalSize / 1024.0f);

        _updatePartition = esp_ota_get_next_update_partition(NULL);
        if (!_updatePartition) {
            Serial.println("[OTA] Error: Failed to find next inactive OTA partition!");
            notifyError(OTA_ERR_PARTITION_NOT_FOUND);
            return;
        }

        Serial.printf("[OTA] Target update partition: %s (offset: 0x%08X, max size: %u bytes)\n",
                      _updatePartition->label, _updatePartition->address, _updatePartition->size);

        if (_totalSize > _updatePartition->size) {
            Serial.printf("[OTA] Error: Firmware size %u exceeds partition capacity %u!\n",
                          _totalSize, _updatePartition->size);
            notifyError(OTA_ERR_BEGIN_FAILED);
            return;
        }

        esp_err_t err = esp_ota_begin(_updatePartition, _totalSize, &_otaHandle);
        if (err != ESP_OK) {
            Serial.printf("[OTA] esp_ota_begin failed: %s (0x%X)\n", esp_err_to_name(err), err);
            notifyError(OTA_ERR_BEGIN_FAILED);
            return;
        }

        // Initialize SHA-256 hashing context for on-the-fly binary validation
        mbedtls_sha256_init(&_sha256Ctx);
        mbedtls_sha256_starts(&_sha256Ctx, 0); // 0 = standard SHA-256
        _sha256Initialized = true;

        _inProgress = true;
        _bytesWritten = 0;

        if (_onStateChange) {
            _onStateChange(true);
        }

        Serial.println("[OTA] Initialized OTA handle & SHA-256 context successfully. Notifying OTA_READY (0x10)...");
        notifyStatus(OTA_RESP_READY);
        return;
    }

    // Command 0x02: OTA_END
    if (cmd == OTA_CMD_END) {
        Serial.println("[OTA] OTA_END received. Finalizing image verification...");

        if (!_inProgress || _otaHandle == 0) {
            Serial.println("[OTA] Error: Received OTA_END but no update is active.");
            notifyError(OTA_ERR_VALIDATION_FAILED);
            return;
        }

        if (_totalSize > 0 && _bytesWritten != _totalSize) {
            Serial.printf("[OTA] Error: Byte count mismatch! Written: %u, Expected: %u\n",
                          _bytesWritten, _totalSize);
            abort();
            notifyError(OTA_ERR_SIZE_MISMATCH);
            return;
        }

        // Cryptographic SHA-256 validation if client supplied checksum (len >= 33)
        if (len >= 33) {
            uint8_t calculatedSha256[32];
            if (_sha256Initialized) {
                mbedtls_sha256_finish(&_sha256Ctx, calculatedSha256);
                mbedtls_sha256_free(&_sha256Ctx);
                _sha256Initialized = false;
            } else {
                memset(calculatedSha256, 0, sizeof(calculatedSha256));
            }

            if (memcmp(calculatedSha256, &data[1], 32) != 0) {
                Serial.println("[OTA] SECURITY ALERT: SHA-256 Checksum mismatch!");
                Serial.print("[OTA] Expected:   ");
                for (int i = 0; i < 32; i++) Serial.printf("%02x", data[1 + i]);
                Serial.println();
                Serial.print("[OTA] Calculated: ");
                for (int i = 0; i < 32; i++) Serial.printf("%02x", calculatedSha256[i]);
                Serial.println();

                abort();
                notifyError(OTA_ERR_CHECKSUM_MISMATCH);
                return;
            }
            Serial.println("[OTA] SHA-256 Cryptographic Checksum verified successfully!");
        } else {
            if (_sha256Initialized) {
                mbedtls_sha256_free(&_sha256Ctx);
                _sha256Initialized = false;
            }
            Serial.println("[OTA] Warning: No SHA-256 checksum provided in OTA_END packet. Proceeding with image validation...");
        }

        esp_err_t err = esp_ota_end(_otaHandle);
        _otaHandle = 0;
        if (err != ESP_OK) {
            Serial.printf("[OTA] esp_ota_end failed: %s (0x%X)\n", esp_err_to_name(err), err);
            _inProgress = false;
            if (_onStateChange) _onStateChange(false);
            notifyError(OTA_ERR_VALIDATION_FAILED);
            return;
        }

        err = esp_ota_set_boot_partition(_updatePartition);
        if (err != ESP_OK) {
            Serial.printf("[OTA] esp_ota_set_boot_partition failed: %s (0x%X)\n", esp_err_to_name(err), err);
            _inProgress = false;
            if (_onStateChange) _onStateChange(false);
            notifyError(OTA_ERR_SET_BOOT_FAILED);
            return;
        }

        _inProgress = false;
        if (_onStateChange) {
            _onStateChange(false);
        }

        Serial.printf("[OTA] *** OTA SUCCESS! Boot partition switched to %s. Restarting in 500ms... ***\n",
                      _updatePartition->label);

        notifyStatus(OTA_RESP_SUCCESS);

        // Schedule delayed device reboot
        delay(500);
        esp_restart();
        return;
    }

    // Command 0x03: OTA_ABORT
    if (cmd == OTA_CMD_ABORT) {
        Serial.println("[OTA] OTA_ABORT command received from client.");
        abort();
        return;
    }

    Serial.printf("[OTA] Unknown control opcode: 0x%02X\n", cmd);
}

void OtaBleService::handleDataWrite(const uint8_t* data, size_t len) {
    if (!_inProgress || _otaHandle == 0) {
        return;
    }

    if (len == 0 || !data) return;

    esp_err_t err = esp_ota_write(_otaHandle, data, len);
    if (err != ESP_OK) {
        Serial.printf("[OTA] esp_ota_write failed at offset %u: %s (0x%X)\n",
                      _bytesWritten, esp_err_to_name(err), err);
        abort();
        notifyError(OTA_ERR_WRITE_FAILED);
        return;
    }

    if (_sha256Initialized) {
        mbedtls_sha256_update(&_sha256Ctx, data, len);
    }

    _bytesWritten += len;

    // Periodic progress logging every ~64KB or when complete
    if ((_bytesWritten % 65536) < len || _bytesWritten == _totalSize) {
        float pct = (_totalSize > 0) ? (100.0f * _bytesWritten / _totalSize) : 0.0f;
        Serial.printf("[OTA] Flashed: %u / %u bytes (%.1f%%)\n", _bytesWritten, _totalSize, pct);
    }
}

void OtaBleService::abort() {
    if (_inProgress && _otaHandle != 0) {
        esp_ota_abort(_otaHandle);
        _otaHandle = 0;
    }
    if (_sha256Initialized) {
        mbedtls_sha256_free(&_sha256Ctx);
        _sha256Initialized = false;
    }
    _inProgress = false;
    _bytesWritten = 0;
    _totalSize = 0;
    _updatePartition = nullptr;

    if (_onStateChange) {
        _onStateChange(false);
    }
    Serial.println("[OTA] OTA session aborted and cleaned up.");
}
