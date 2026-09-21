#pragma once

#include <stdint.h>

/**
 * @file invariants.h
 * @brief HealthKicks BLE GATT profile specifications and constants.
 * Contract reference: contracts/ble_gatt_specs.md
 */

// 128-bit UUIDs derived from base 7a5a0000-c529-4d64-8848-18e5904de22a
#define HEALTHKICKS_SERVICE_UUID        "7a5a0001-c529-4d64-8848-18e5904de22a"
#define CHAR_ACTIVITY_DETECTION_UUID    "7a5a0002-c529-4d64-8848-18e5904de22a"
#define CHAR_HAPTIC_COMMAND_UUID        "7a5a0003-c529-4d64-8848-18e5904de22a"
#define CHAR_STUDIO_CONTROL_UUID        "7a5a0004-c529-4d64-8848-18e5904de22a"
#define CHAR_STUDIO_DATA_BURST_UUID     "7a5a0005-c529-4d64-8848-18e5904de22a"

// Activity state codes (Characteristic 0002)
#define STATE_CODE_IDLE                 0x00
#define STATE_CODE_WALK                 0x01
#define STATE_CODE_RUN                  0x02
#define STATE_CODE_STAIRS               0x03
#define STATE_CODE_FALL_FORWARD         0x10
#define STATE_CODE_FALL_BACKWARD        0x11
#define STATE_CODE_FALL_LATERAL         0x12
#define STATE_CODE_STUMBLE_RECOVER      0x1E
#define STATE_CODE_FALL_GENERIC         0x1F

// Detection flags (Characteristic 0002)
#define DETECTION_FLAG_CRITICAL_FALL    (1 << 0)
#define DETECTION_FLAG_LOCAL_HAPTIC     (1 << 1)

// Packet types for Studio Burst Transfer (Characteristic 0005)
#define BURST_PACKET_START_OF_BURST     0x01
#define BURST_PACKET_DATA_CHUNK         0x02
#define BURST_PACKET_END_OF_BURST       0x03

// Haptic vibration patterns & Control Commands (Characteristic 0003)
#define HAPTIC_PATTERN_CONTINUOUS       0
#define HAPTIC_PATTERN_DOUBLE_PULSE     1
#define HAPTIC_PATTERN_ALERT_PULSE      2
#define CMD_TRIGGER_CALIBRATION         0x05

// IMU frame parameters
#define IMU_BYTES_PER_FRAME             14
#define IMU_NOMINAL_SAMPLE_RATE_HZ      50

#pragma pack(push, 1)
/**
 * @brief Compressed 14-byte IMU raw frame structure (Big-Endian).
 */
struct ImuRawFrame {
    uint16_t delta_ms;  // Elapsed time in ms since session start
    int16_t ax;         // X Acceleration in milli-g (x1000)
    int16_t ay;         // Y Acceleration in milli-g (x1000)
    int16_t az;         // Z Acceleration in milli-g (x1000)
    int16_t gx;         // X Angular velocity in tenths of deg/s (x10)
    int16_t gy;         // Y Angular velocity in tenths of deg/s (x10)
    int16_t gz;         // Z Angular velocity in tenths of deg/s (x10)
};

/**
 * @brief Standard 4-byte Burst Transfer packet header.
 */
struct BurstPacketHeader {
    uint8_t packet_type;    // 0x01 = START, 0x02 = DATA_CHUNK, 0x03 = END
    uint16_t seq_num;       // Sequence number (Big-Endian)
    uint8_t payload_len;    // Sample count or payload length
};
#pragma pack(pop)

