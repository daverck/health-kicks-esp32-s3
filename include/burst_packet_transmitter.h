#pragma once

#include <Arduino.h>
#include <vector>
#include <stdint.h>
#include <stddef.h>
#include "invariants.h"
#include "ble/ble_server.h"

/**
 * @file burst_packet_transmitter.h
 * @brief High-throughput BLE burst packet encoder & transmitter with RLE/Delta compression and CRC32 verification.
 */
class BurstPacketTransmitter {
public:
    /**
     * @brief Computes standard IEEE 802.3 CRC32 checksum over a binary buffer.
     * @param data Pointer to binary data.
     * @param length Number of bytes.
     * @return 32-bit CRC32 checksum.
     */
    static uint32_t computeCrc32(const uint8_t* data, size_t length);

    /**
     * @brief Transmits a vector of raw IMU frames across fragmented BLE packets (RLE, Delta, Raw, and End).
     * @param bleServer Active BLE server instance.
     * @param frames Buffer of raw IMU samples.
     * @return Number of packets transmitted, or 0 if aborted/disconnected.
     */
    static uint16_t sendBurstPackets(HealthKicksBleServer* bleServer, const std::vector<ImuRawFrame>& frames);
};
