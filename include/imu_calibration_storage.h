#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @file imu_calibration_storage.h
 * @brief NVS Flash persistence helper for IMU orientation matrix and gyroscope bias.
 */
class ImuCalibrationStorage {
public:
    static constexpr const char* DEFAULT_NAMESPACE = "hk_calib_v2";

    explicit ImuCalibrationStorage(const char* ns = DEFAULT_NAMESPACE);

    /**
     * @brief Saves 3x3 rotation matrix, gyroscope bias vector, and calibration step into NVS flash.
     * @param r 3x3 rotation matrix.
     * @param gbx Gyroscope X bias in deg/s.
     * @param gby Gyroscope Y bias in deg/s.
     * @param gbz Gyroscope Z bias in deg/s.
     * @param step Calibration step index (1 or 2).
     * @return true if successfully saved.
     */
    bool saveCalibration(const float r[3][3], float gbx, float gby, float gbz, int step);

    /**
     * @brief Loads 3x3 rotation matrix and gyroscope bias vector from NVS flash if valid.
     * @param[out] r 3x3 rotation matrix to populate.
     * @param[out] gbx Gyroscope X bias in deg/s.
     * @param[out] gby Gyroscope Y bias in deg/s.
     * @param[out] gbz Gyroscope Z bias in deg/s.
     * @return true if valid calibration parameters were found and loaded.
     */
    bool loadCalibration(float r[3][3], float& gbx, float& gby, float& gbz);

    /**
     * @brief Clears stored calibration parameters in NVS flash.
     */
    void clear();

private:
    const char* _ns;
    Preferences _prefs;
};
