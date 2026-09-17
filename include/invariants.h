#pragma once

#include <stdint.h>

/**
 * @file invariants.h
 * @brief Identifiants et spécifications de profil BLE GATT HealthKicks.
 * Référence contractuelle : contracts/ble_gatt_specs.md
 */

// UUIDs 128-bit dérivés de la base 7a5a0000-c529-4d64-8848-18e5904de22a
#define HEALTHKICKS_SERVICE_UUID        "7a5a0001-c529-4d64-8848-18e5904de22a"
#define CHAR_ACTIVITY_DETECTION_UUID    "7a5a0002-c529-4d64-8848-18e5904de22a"
#define CHAR_HAPTIC_COMMAND_UUID        "7a5a0003-c529-4d64-8848-18e5904de22a"
#define CHAR_STUDIO_CONTROL_UUID        "7a5a0004-c529-4d64-8848-18e5904de22a"
#define CHAR_STUDIO_DATA_BURST_UUID     "7a5a0005-c529-4d64-8848-18e5904de22a"

// Codes d'état d'activité (Caractéristique 0002)
#define STATE_CODE_IDLE                 0x00
#define STATE_CODE_WALK                 0x01
#define STATE_CODE_RUN                  0x02
#define STATE_CODE_FALL_FORWARD         0x10
#define STATE_CODE_FALL_BACKWARD        0x11
#define STATE_CODE_FALL_LATERAL         0x12
#define STATE_CODE_FALL_GENERIC         0x1F

// Flags de détection (Caractéristique 0002)
#define DETECTION_FLAG_CRITICAL_FALL    (1 << 0)
#define DETECTION_FLAG_LOCAL_HAPTIC     (1 << 1)

// Types de paquets pour le Burst Transfer Studio (Caractéristique 0005)
#define BURST_PACKET_START_OF_BURST     0x01
#define BURST_PACKET_DATA_CHUNK         0x02
#define BURST_PACKET_END_OF_BURST       0x03

// Patterns de vibration haptique (Caractéristique 0003)
#define HAPTIC_PATTERN_CONTINUOUS       0
#define HAPTIC_PATTERN_DOUBLE_PULSE     1
#define HAPTIC_PATTERN_ALERT_PULSE      2

// Paramètres de trame IMU
#define IMU_BYTES_PER_FRAME             14
#define IMU_NOMINAL_SAMPLE_RATE_HZ      50

#pragma pack(push, 1)
/**
 * @brief Structure d'une trame IMU compressée de 14 octets (Big-Endian).
 */
struct ImuRawFrame {
    uint16_t delta_ms;  // Temps écoulé en ms depuis le début de session
    int16_t ax;         // Accélération X en milli-g (x1000)
    int16_t ay;         // Accélération Y en milli-g (x1000)
    int16_t az;         // Accélération Z en milli-g (x1000)
    int16_t gx;         // Vitesse angulaire X en dixièmes de deg/s (x10)
    int16_t gy;         // Vitesse angulaire Y en dixièmes de deg/s (x10)
    int16_t gz;         // Vitesse angulaire Z en dixièmes de deg/s (x10)
};

/**
 * @brief En-tête standard de paquet Burst Transfer (4 octets).
 */
struct BurstPacketHeader {
    uint8_t packet_type;    // 0x01 = START, 0x02 = DATA_CHUNK, 0x03 = END
    uint16_t seq_num;       // Numéro séquentiel (Big-Endian)
    uint8_t payload_len;    // Nombre d'échantillons ou taille
};
#pragma pack(pop)

