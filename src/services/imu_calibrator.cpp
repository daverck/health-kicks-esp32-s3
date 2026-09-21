#include "imu_calibrator.h"

static const char* NVS_NAMESPACE = "hk_calib";

ImuCalibrator::ImuCalibrator()
    : _phase(CALIB_WAITING_INITIAL_IDLE),
      _isCalibrated(false),
      _manualCalibrationPending(false),
      _sampleRateHz(DEFAULT_SAMPLE_RATE_HZ),
      _durationSec(DEFAULT_CALIBRATION_IDLE_DURATION_SEC),
      _requiredSamples(76),
      _stillnessSampleCount(0),
      _sumAx(0.0),
      _sumAy(0.0),
      _sumAz(0.0),
      _onComplete(nullptr) {
    // Initialize rotation matrix to identity
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            _r[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void ImuCalibrator::begin(uint16_t sampleRateHz, float durationSec) {
    _sampleRateHz = sampleRateHz > 0 ? sampleRateHz : DEFAULT_SAMPLE_RATE_HZ;
    setStillnessDurationSec(durationSec);

    // Try loading persistent calibration from NVS
    if (loadFromNvs()) {
        _isCalibrated = true;
        _phase = CALIB_FIRST_DONE_WAITING_WALK;
        Serial.println("[CALIB] Restored existing sensor orientation matrix from NVS.");
    } else {
        _isCalibrated = false;
        _phase = CALIB_WAITING_INITIAL_IDLE;
        Serial.println("[CALIB] No stored calibration found in NVS. Waiting for initial level rest...");
    }

    _stillnessSampleCount = 0;
    _sumAx = 0.0;
    _sumAy = 0.0;
    _sumAz = 0.0;
    _manualCalibrationPending = false;
}

void ImuCalibrator::setStillnessDurationSec(float sec) {
    _durationSec = (sec >= 1.0f && sec <= 20.0f) ? sec : DEFAULT_CALIBRATION_IDLE_DURATION_SEC;
    _requiredSamples = (uint32_t)round(_durationSec * _sampleRateHz);
    if (_requiredSamples < 10) _requiredSamples = 10;
}

bool ImuCalibrator::checkStillness(float ax, float ay, float az, float gx, float gy, float gz) const {
    // 1. Gyroscope stillness: magnitude < 3.0 deg/s
    float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
    if (gyroMag > MAX_GYRO_STILLNESS_DPS) {
        return false;
    }

    // 2. Accelerometer stillness: magnitude within [0.9g, 1.1g]
    float accelMag = sqrtf(ax * ax + ay * ay + az * az);
    if (accelMag < MIN_ACCEL_STILLNESS_G || accelMag > MAX_ACCEL_STILLNESS_G) {
        return false;
    }

    // 3. Roughly horizontal resting orientation: Az > 0.7g
    if (az < MIN_ROUGH_HORIZONTAL_AZ_G) {
        return false;
    }

    return true;
}

void ImuCalibrator::update(float ax_raw, float ay_raw, float az_raw, float gx_raw, float gy_raw, float gz_raw) {
    // Determine whether calibration accumulation is needed
    bool shouldCalibrate = _manualCalibrationPending ||
                           (_phase == CALIB_WAITING_INITIAL_IDLE) ||
                           (_phase == CALIB_WALK_DETECTED_WAITING_SECOND_IDLE);

    if (!shouldCalibrate) {
        return;
    }

    // Check if the current raw reading meets physical stillness and horizontal criteria
    if (checkStillness(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
        _sumAx += (double)ax_raw;
        _sumAy += (double)ay_raw;
        _sumAz += (double)az_raw;
        _stillnessSampleCount++;

        if (_stillnessSampleCount >= _requiredSamples) {
            int step = _manualCalibrationPending ? 0 : (_phase == CALIB_WAITING_INITIAL_IDLE ? 1 : 2);
            computeAndApplyAlignment(step);
            saveToNvs(step);

            if (_manualCalibrationPending) {
                _manualCalibrationPending = false;
                Serial.println("[CALIB] Manual zero-calibration completed and persisted to NVS.");
            } else if (_phase == CALIB_WAITING_INITIAL_IDLE) {
                _phase = CALIB_FIRST_DONE_WAITING_WALK;
                Serial.println("[CALIB] Step 1: Initial level calibration applied. Ax/Ay -> ~0.0g, Az -> ~1.0g");
            } else if (_phase == CALIB_WALK_DETECTED_WAITING_SECOND_IDLE) {
                _phase = CALIB_COMPLETED;
                Serial.println("[CALIB] Step 2: Post-walk refinement calibration applied and saved to NVS.");
            }

            // Reset sample accumulator
            _stillnessSampleCount = 0;
            _sumAx = 0.0;
            _sumAy = 0.0;
            _sumAz = 0.0;

            // Notify completion listener (e.g. main.cpp for haptic feedback and BLE notification)
            if (_onComplete) {
                _onComplete(true, step);
            }
        }
    } else {
        // Motion detected: reset continuous stillness accumulator
        if (_stillnessSampleCount > 0) {
            _stillnessSampleCount = 0;
            _sumAx = 0.0;
            _sumAy = 0.0;
            _sumAz = 0.0;
        }
    }
}

void ImuCalibrator::computeAndApplyAlignment(int step) {
    if (_stillnessSampleCount == 0) return;

    // Compute averaged gravity vector in raw sensor frame
    double invN = 1.0 / (double)_stillnessSampleCount;
    double vx = _sumAx * invN;
    double vy = _sumAy * invN;
    double vz = _sumAz * invN;

    double mag = sqrt(vx * vx + vy * vy + vz * vz);
    if (mag < 1e-4) return;

    // Normalized measured gravity vector
    float nx = (float)(vx / mag);
    float ny = (float)(vy / mag);
    float nz = (float)(vz / mag);

    // Target vertical gravity vector in shoe body frame: v_target = [0, 0, 1.0]
    // Rotation axis: u = v_meas x v_target = [ny, -nx, 0]
    // Cosine of angle: c = v_meas . v_target = nz
    float s = sqrtf(nx * nx + ny * ny);

    if (s < 1e-5f || (1.0f + nz) < 1e-5f) {
        // Already aligned or antiparallel safety fallback
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                _r[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    } else {
        // Rodrigues' rotation formula: R = I + [u]_x + [u]_x^2 / (1 + c)
        float k = 1.0f / (1.0f + nz);

        _r[0][0] = 1.0f - k * nx * nx;
        _r[0][1] = -k * nx * ny;
        _r[0][2] = -nx;

        _r[1][0] = -k * nx * ny;
        _r[1][1] = 1.0f - k * ny * ny;
        _r[1][2] = -ny;

        _r[2][0] = nx;
        _r[2][1] = ny;
        _r[2][2] = nz;
    }

    _isCalibrated = true;
    Serial.printf("[CALIB] Calculated rotation matrix R (step %d) from measured gravity [%.3f, %.3f, %.3f]:\n",
                  step, nx, ny, nz);
    Serial.printf("        [ %.4f, %.4f, %.4f ]\n", _r[0][0], _r[0][1], _r[0][2]);
    Serial.printf("        [ %.4f, %.4f, %.4f ]\n", _r[1][0], _r[1][1], _r[1][2]);
    Serial.printf("        [ %.4f, %.4f, %.4f ]\n", _r[2][0], _r[2][1], _r[2][2]);
}

void ImuCalibrator::applyCalibration(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) const {
    if (!_isCalibrated) {
        return;
    }

    // Apply rotation matrix R to linear acceleration vector
    float rx = _r[0][0] * ax + _r[0][1] * ay + _r[0][2] * az;
    float ry = _r[1][0] * ax + _r[1][1] * ay + _r[1][2] * az;
    float rz = _r[2][0] * ax + _r[2][1] * ay + _r[2][2] * az;
    ax = rx;
    ay = ry;
    az = rz;

    // Apply the exact same rotation matrix R to angular velocity vector (gyroscope)
    float rgx = _r[0][0] * gx + _r[0][1] * gy + _r[0][2] * gz;
    float rgy = _r[1][0] * gx + _r[1][1] * gy + _r[1][2] * gz;
    float rgz = _r[2][0] * gx + _r[2][1] * gy + _r[2][2] * gz;
    gx = rgx;
    gy = rgy;
    gz = rgz;
}

void ImuCalibrator::notifyWalkDetected() {
    if (_phase == CALIB_FIRST_DONE_WAITING_WALK) {
        _phase = CALIB_WALK_DETECTED_WAITING_SECOND_IDLE;
        _stillnessSampleCount = 0;
        _sumAx = 0.0;
        _sumAy = 0.0;
        _sumAz = 0.0;
        Serial.println("[CALIB] Walking activity detected. Next resting stop will trigger step 2 refinement calibration.");
    }
}

void ImuCalibrator::triggerManualCalibration() {
    _manualCalibrationPending = true;
    _stillnessSampleCount = 0;
    _sumAx = 0.0;
    _sumAy = 0.0;
    _sumAz = 0.0;
    Serial.println("[CALIB] Manual calibration sequence triggered via BLE. Keep shoe flat and still for 4 seconds...");
}

void ImuCalibrator::saveToNvs(int step) {
    if (!_prefs.begin(NVS_NAMESPACE, false)) {
        log_e("Failed to open NVS namespace %s for writing", NVS_NAMESPACE);
        return;
    }

    _prefs.putBool("valid", true);
    _prefs.putInt("step", step);
    _prefs.putFloat("r00", _r[0][0]);
    _prefs.putFloat("r01", _r[0][1]);
    _prefs.putFloat("r02", _r[0][2]);
    _prefs.putFloat("r10", _r[1][0]);
    _prefs.putFloat("r11", _r[1][1]);
    _prefs.putFloat("r12", _r[1][2]);
    _prefs.putFloat("r20", _r[2][0]);
    _prefs.putFloat("r21", _r[2][1]);
    _prefs.putFloat("r22", _r[2][2]);
    _prefs.end();

    log_i("Saved calibration matrix (step %d) to NVS namespace %s", step, NVS_NAMESPACE);
}

bool ImuCalibrator::loadFromNvs() {
    if (!_prefs.begin(NVS_NAMESPACE, true)) {
        return false;
    }

    bool isValid = _prefs.getBool("valid", false);
    if (!isValid) {
        _prefs.end();
        return false;
    }

    _r[0][0] = _prefs.getFloat("r00", 1.0f);
    _r[0][1] = _prefs.getFloat("r01", 0.0f);
    _r[0][2] = _prefs.getFloat("r02", 0.0f);

    _r[1][0] = _prefs.getFloat("r10", 0.0f);
    _r[1][1] = _prefs.getFloat("r11", 1.0f);
    _r[1][2] = _prefs.getFloat("r12", 0.0f);

    _r[2][0] = _prefs.getFloat("r20", 0.0f);
    _r[2][1] = _prefs.getFloat("r21", 0.0f);
    _r[2][2] = _prefs.getFloat("r22", 1.0f);

    _prefs.end();
    return true;
}

void ImuCalibrator::reset() {
    if (_prefs.begin(NVS_NAMESPACE, false)) {
        _prefs.clear();
        _prefs.end();
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            _r[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    _isCalibrated = false;
    _phase = CALIB_WAITING_INITIAL_IDLE;
    _manualCalibrationPending = false;
    _stillnessSampleCount = 0;
    _sumAx = 0.0;
    _sumAy = 0.0;
    _sumAz = 0.0;
    Serial.println("[CALIB] NVS calibration cleared. Calibrator reset to initial identity state.");
}

