#pragma once

#include <stdint.h>
#include <stddef.h>
#include "invariants.h"

// Forward declaration of BLE server
class HealthKicksBleServer;

/**
 * @brief Represents an individual offline fall event stored in non-volatile flash.
 */
struct OfflineFallRecord {
    uint8_t stateCode;
    uint8_t confidence;
    uint32_t timestampSec;
};

/**
 * @brief Thread-safe non-volatile storage manager for fall alerts detected while BLE is disconnected.
 *
 * Employs a circular buffer in NVS (Non-Volatile Storage) to ensure that emergency fall alerts
 * are never dropped due to Bluetooth disconnections or unexpected reboots.
 */
class OfflineFallStorage {
public:
    static constexpr uint8_t MAX_FALLS = 10;

    OfflineFallStorage();
    ~OfflineFallStorage();

    /**
     * @brief Initialize NVS namespace and load metadata.
     */
    bool begin();

    /**
     * @brief Persist an offline fall event to flash.
     * @param stateCode Activity state code (e.g. STATE_CODE_FALL_FORWARD).
     * @param confidence Detection confidence in percent (0-100).
     * @param timestampSec Unix timestamp (or uptime seconds) when fall occurred.
     * @return true if successfully saved.
     */
    bool saveFall(uint8_t stateCode, uint8_t confidence, uint32_t timestampSec);

    /**
     * @brief Check whether any unread offline fall events remain in flash.
     */
    bool hasPendingFalls() const;

    /**
     * @brief Get count of pending offline fall events.
     */
    uint8_t getPendingCount() const;

    /**
     * @brief Read and transmit all pending falls over BLE to the reconnected mobile app, then clear storage.
     * @param bleServer Reference to the initialized HealthKicksBleServer.
     * @return Number of flushed fall events.
     */
    uint8_t flushToBle(HealthKicksBleServer& bleServer);

    /**
     * @brief Clear all persisted offline falls from NVS.
     */
    void clear();

private:
    uint8_t _count;
    uint8_t _head;
    bool _initialized;
};

