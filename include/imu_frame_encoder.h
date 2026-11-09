#pragma once

#include <stdint.h>
#include <cmath>
#include <lwip/def.h>
#include "invariants.h"

/**
 * @file imu_frame_encoder.h
 * @brief Standardized quantization and network byte-order (Big-Endian) encoding
 * for 14-byte ImuRawFrame structures.
 */

/**
 * @brief Quantizes physical units into a packed 14-byte Big-Endian ImuRawFrame.
 *
 * @param elapsedMs Elapsed time in milliseconds since reference.
 * @param ax_g X Acceleration in g (scaled to milli-g x1000).
 * @param ay_g Y Acceleration in g (scaled to milli-g x1000).
 * @param az_g Z Acceleration in g (scaled to milli-g x1000).
 * @param gx_dps X Angular velocity in deg/s (scaled to tenths of deg/s x10).
 * @param gy_dps Y Angular velocity in deg/s (scaled to tenths of deg/s x10).
 * @param gz_dps Z Angular velocity in deg/s (scaled to tenths of deg/s x10).
 * @return Encoded ImuRawFrame ready for network burst transmission or buffer storage.
 */
static inline ImuRawFrame encodeImuRawFrame(
    uint16_t elapsedMs,
    float ax_g, float ay_g, float az_g,
    float gx_dps, float gy_dps, float gz_dps
) {
    ImuRawFrame frame;
    frame.delta_ms = htons(elapsedMs);
    frame.ax = htons((int16_t)roundf(ax_g * 1000.0f));
    frame.ay = htons((int16_t)roundf(ay_g * 1000.0f));
    frame.az = htons((int16_t)roundf(az_g * 1000.0f));
    frame.gx = htons((int16_t)roundf(gx_dps * 10.0f));
    frame.gy = htons((int16_t)roundf(gy_dps * 10.0f));
    frame.gz = htons((int16_t)roundf(gz_dps * 10.0f));
    return frame;
}
