#pragma once

#include <stdint.h>
#include <cmath>
#include <lwip/def.h>
#include "invariants.h"

/**
 * @file imu_frame_encoder.h
 * @brief Standardized quantization, network byte-order (Big-Endian) encoding,
 * and physical unit decoding for 14-byte ImuRawFrame structures.
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

/**
 * @brief Decodes a packed 14-byte Big-Endian ImuRawFrame back into physical units.
 *
 * @param frame Packed ImuRawFrame in network byte order.
 * @param[out] ax_g Acceleration along X-axis in g.
 * @param[out] ay_g Acceleration along Y-axis in g.
 * @param[out] az_g Acceleration along Z-axis in g.
 * @param[out] gx_dps Angular velocity along X-axis in deg/s.
 * @param[out] gy_dps Angular velocity along Y-axis in deg/s.
 * @param[out] gz_dps Angular velocity along Z-axis in deg/s.
 */
static inline void decodeImuRawFrame(
    const ImuRawFrame& frame,
    float& ax_g, float& ay_g, float& az_g,
    float& gx_dps, float& gy_dps, float& gz_dps
) {
    ax_g = (float)((int16_t)ntohs(frame.ax)) / 1000.0f;
    ay_g = (float)((int16_t)ntohs(frame.ay)) / 1000.0f;
    az_g = (float)((int16_t)ntohs(frame.az)) / 1000.0f;

    gx_dps = (float)((int16_t)ntohs(frame.gx)) / 10.0f;
    gy_dps = (float)((int16_t)ntohs(frame.gy)) / 10.0f;
    gz_dps = (float)((int16_t)ntohs(frame.gz)) / 10.0f;
}
