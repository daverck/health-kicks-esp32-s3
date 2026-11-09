#include "imu_calibration_storage.h"

ImuCalibrationStorage::ImuCalibrationStorage(const char* ns)
    : _ns(ns) {
}

bool ImuCalibrationStorage::saveCalibration(const float r[3][3], float gbx, float gby, float gbz, int step) {
    if (!_prefs.begin(_ns, false)) {
        log_e("Failed to open NVS namespace %s for writing", _ns);
        return false;
    }

    _prefs.putBool("valid", true);
    _prefs.putInt("step", step);
    _prefs.putFloat("r00", r[0][0]);
    _prefs.putFloat("r01", r[0][1]);
    _prefs.putFloat("r02", r[0][2]);
    _prefs.putFloat("r10", r[1][0]);
    _prefs.putFloat("r11", r[1][1]);
    _prefs.putFloat("r12", r[1][2]);
    _prefs.putFloat("r20", r[2][0]);
    _prefs.putFloat("r21", r[2][1]);
    _prefs.putFloat("r22", r[2][2]);
    _prefs.putFloat("gbx", gbx);
    _prefs.putFloat("gby", gby);
    _prefs.putFloat("gbz", gbz);
    _prefs.end();

    log_i("Saved calibration matrix (step %d) and gyro bias [%.2f, %.2f, %.2f] to NVS namespace %s",
          step, gbx, gby, gbz, _ns);
    return true;
}

bool ImuCalibrationStorage::loadCalibration(float r[3][3], float& gbx, float& gby, float& gbz) {
    if (!_prefs.begin(_ns, true)) {
        return false;
    }

    bool isValid = _prefs.getBool("valid", false);
    if (!isValid || !_prefs.isKey("gbx") || !_prefs.isKey("gby") || !_prefs.isKey("gbz")) {
        _prefs.end();
        return false;
    }

    r[0][0] = _prefs.getFloat("r00", 1.0f);
    r[0][1] = _prefs.getFloat("r01", 0.0f);
    r[0][2] = _prefs.getFloat("r02", 0.0f);

    r[1][0] = _prefs.getFloat("r10", 0.0f);
    r[1][1] = _prefs.getFloat("r11", 1.0f);
    r[1][2] = _prefs.getFloat("r12", 0.0f);

    r[2][0] = _prefs.getFloat("r20", 0.0f);
    r[2][1] = _prefs.getFloat("r21", 0.0f);
    r[2][2] = _prefs.getFloat("r22", 1.0f);

    gbx = _prefs.getFloat("gbx", 0.0f);
    gby = _prefs.getFloat("gby", 0.0f);
    gbz = _prefs.getFloat("gbz", 0.0f);

    _prefs.end();
    return true;
}

void ImuCalibrationStorage::clear() {
    if (_prefs.begin(_ns, false)) {
        _prefs.clear();
        _prefs.end();
    }
}
