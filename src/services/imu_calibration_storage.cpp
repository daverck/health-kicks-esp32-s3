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

    char key[4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            snprintf(key, sizeof(key), "r%d%d", i, j);
            _prefs.putFloat(key, r[i][j]);
        }
    }

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

    char key[4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            snprintf(key, sizeof(key), "r%d%d", i, j);
            r[i][j] = _prefs.getFloat(key, (i == j) ? 1.0f : 0.0f);
        }
    }

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
