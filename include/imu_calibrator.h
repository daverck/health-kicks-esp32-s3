#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <Preferences.h>
#include <functional>

/**
 * @file imu_calibrator.h
 * @brief Dynamic IMU orientation and tilt alignment engine for ESP32-S3.
 * 
 * Automatically corrects physical sensor misalignment on the footwear
 * using a 2-step process (initial at boot, refinement post-walk) and supports
 * on-demand manual zero-calibration via BLE commands.
 */

enum CalibrationPhase {
    CALIB_WAITING_INITIAL_IDLE,                 // Boot phase: waiting for initial level stillness
    CALIB_FIRST_DONE_WAITING_WALK,              // Step 1 completed: waiting for walk activity
    CALIB_WALK_DETECTED_WAITING_SECOND_IDLE,    // Walk occurred: waiting for post-walk rest stop
    CALIB_COMPLETED                             // 2-step automatic calibration finished
};

typedef std::function<void(bool success, int step)> CalibrationCompleteCallback;

class ImuCalibrator {
public:
    static constexpr float DEFAULT_CALIBRATION_IDLE_DURATION_SEC = 4.0f;
    static constexpr float MAX_GYRO_STILLNESS_DPS = 25.0f;       // Tolerates MPU-6050 zero-rate drift at rest
    static constexpr float MIN_ACCEL_STILLNESS_G = 0.80f;
    static constexpr float MAX_ACCEL_STILLNESS_G = 1.20f;
    static constexpr float MIN_ROUGH_HORIZONTAL_AZ_G = 0.60f;     // Rough horizontal level requirement
    static constexpr uint16_t DEFAULT_SAMPLE_RATE_HZ = 19;

    ImuCalibrator();

    /**
     * @brief Initializes the calibrator, loads stored calibration from NVS if present.
     * @param sampleRateHz Nominal IMU sampling rate (default: 19 Hz).
     * @param durationSec Duration of stillness required to confirm calibration (default: 4.0s).
     */
    void begin(uint16_t sampleRateHz = DEFAULT_SAMPLE_RATE_HZ, float durationSec = DEFAULT_CALIBRATION_IDLE_DURATION_SEC);

    /**
     * @brief Ingests an uncalibrated physical IMU sample and updates calibration state machine.
     * @param ax_raw Raw acceleration X in g.
     * @param ay_raw Raw acceleration Y in g.
     * @param az_raw Raw acceleration Z in g.
     * @param gx_raw Raw angular velocity X in deg/s.
     * @param gy_raw Raw angular velocity Y in deg/s.
     * @param gz_raw Raw angular velocity Z in deg/s.
     */
    void update(float ax_raw, float ay_raw, float az_raw, float gx_raw, float gy_raw, float gz_raw);

    /**
     * @brief Transforms 3D acceleration and gyroscope vectors using the active rotation matrix.
     * @param[in,out] ax Acceleration X in g.
     * @param[in,out] ay Acceleration Y in g.
     * @param[in,out] az Acceleration Z in g.
     * @param[in,out] gx Angular velocity X in deg/s.
     * @param[in,out] gy Angular velocity Y in deg/s.
     * @param[in,out] gz Angular velocity Z in deg/s.
     */
    void applyCalibration(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) const;

    /**
     * @brief Notifies the calibrator that a walking activity has been detected.
     * Transitions from CALIB_FIRST_DONE_WAITING_WALK to CALIB_WALK_DETECTED_WAITING_SECOND_IDLE.
     */
    void notifyWalkDetected();

    /**
     * @brief Triggers an immediate manual calibration request (via BLE).
     */
    void triggerManualCalibration();

    /**
     * @brief Registers callback invoked upon successful calibration completion.
     */
    void setOnCalibrationComplete(CalibrationCompleteCallback cb) { _onComplete = cb; }

    /**
     * @brief Returns true if an alignment matrix is currently active.
     */
    bool isCalibrated() const { return _isCalibrated; }

    /**
     * @brief Returns current calibration phase.
     */
    CalibrationPhase getPhase() const { return _phase; }

    /**
     * @brief Sets duration of required stillness in seconds.
     */
    void setStillnessDurationSec(float sec);

    /**
     * @brief Clears stored NVS calibration and resets rotation matrix to identity.
     */
    void reset();

    /**
     * @brief Exports the 3x3 rotation matrix as a flat 9-element array (for RTC memory backup).
     */
    void getRotationMatrixFlat(float matrix9[9]) const;

    /**
     * @brief Restores the 3x3 rotation matrix from a flat 9-element array (from RTC memory).
     */
    void restoreFromMatrix(const float matrix9[9]);

private:
    CalibrationPhase _phase;
    bool _isCalibrated;
    bool _manualCalibrationPending;
    uint16_t _sampleRateHz;
    float _durationSec;
    uint32_t _requiredSamples;

    // Accumulation ring for gravity alignment
    uint32_t _stillnessSampleCount;
    double _sumAx;
    double _sumAy;
    double _sumAz;

    // 3x3 Rotation matrix aligning measured gravity to target vertical [0, 0, 1]
    float _r[3][3];

    CalibrationCompleteCallback _onComplete;
    Preferences _prefs;

    bool checkStillness(float ax, float ay, float az, float gx, float gy, float gz) const;
    void computeAndApplyAlignment(int step);
    void saveToNvs(int step);
    bool loadFromNvs();
};

