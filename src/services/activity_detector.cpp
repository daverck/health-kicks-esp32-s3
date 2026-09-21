#include "activity_detector.h"

// Earth gravity constant for converting acceleration between g and m/s^2
static const float GRAVITY_MS2 = 9.80665f;

ActivityDetector::ActivityDetector()
    : _writeIndex(0),
      _sampleCount(0),
      _samplesSinceLastEval(0),
      _confidenceThreshold(0.75f),
      _fallCooldownMs(5000),
      _minFallImpactMs2(18.0f),
      _lastFallTimestampMs(0) {
    memset(_buffer, 0, sizeof(_buffer));
}

void ActivityDetector::begin(float confidenceThreshold, uint32_t fallCooldownMs, float minFallImpactMs2) {
    _confidenceThreshold = confidenceThreshold;
    _fallCooldownMs = fallCooldownMs;
    _minFallImpactMs2 = minFallImpactMs2;
    reset();
}

void ActivityDetector::reset() {
    _writeIndex = 0;
    _sampleCount = 0;
    _samplesSinceLastEval = 0;
    memset(_buffer, 0, sizeof(_buffer));
}

void ActivityDetector::pushSample(float ax, float ay, float az, float gx, float gy, float gz) {
    _buffer[_writeIndex].ax = ax;
    _buffer[_writeIndex].ay = ay;
    _buffer[_writeIndex].az = az;
    _buffer[_writeIndex].gx = gx;
    _buffer[_writeIndex].gy = gy;
    _buffer[_writeIndex].gz = gz;

    _writeIndex = (_writeIndex + 1) % MODEL_WINDOW_SAMPLES;
    if (_sampleCount < MODEL_WINDOW_SAMPLES) {
        _sampleCount++;
    }
    _samplesSinceLastEval++;
}

void ActivityDetector::pushFrame(const ImuRawFrame& frame) {
    // Convert Big-Endian compressed frame:
    // ax, ay, az: milli-g (x1000) -> float in g
    // gx, gy, gz: tenths of deg/s (x10) -> float in deg/s
    int16_t raw_ax = (int16_t)ntohs(frame.ax);
    int16_t raw_ay = (int16_t)ntohs(frame.ay);
    int16_t raw_az = (int16_t)ntohs(frame.az);

    int16_t raw_gx = (int16_t)ntohs(frame.gx);
    int16_t raw_gy = (int16_t)ntohs(frame.gy);
    int16_t raw_gz = (int16_t)ntohs(frame.gz);

    float ax = (float)raw_ax / 1000.0f;
    float ay = (float)raw_ay / 1000.0f;
    float az = (float)raw_az / 1000.0f;

    float gx = (float)raw_gx / 10.0f;
    float gy = (float)raw_gy / 10.0f;
    float gz = (float)raw_gz / 10.0f;

    pushSample(ax, ay, az, gx, gy, gz);
}

bool ActivityDetector::isEvaluationDue() const {
    if (_sampleCount < MODEL_WINDOW_SAMPLES) {
        return false;
    }
    // 50% overlap evaluation step (e.g. every 19 samples for a 38-sample window at 19 Hz)
    const uint32_t stepSamples = (MODEL_WINDOW_SAMPLES > 1) ? (MODEL_WINDOW_SAMPLES / 2) : 1;
    return _samplesSinceLastEval >= stepSamples;
}

void ActivityDetector::computeFeatures(double* featureVector) {
    const uint16_t N = MODEL_WINDOW_SAMPLES;

    double sum_acc_mag = 0.0;
    double sum_acc_mag_sq = 0.0;
    double acc_mag_max = 0.0;
    double acc_mag_min = 1e9;

    double sum_gyro_mag = 0.0;
    double sum_gyro_mag_sq = 0.0;
    double gyro_mag_max = 0.0;

    double sum_ax = 0.0, sum_ax_sq = 0.0;
    double sum_ay = 0.0, sum_ay_sq = 0.0;
    double sum_az = 0.0, sum_az_sq = 0.0;

    double sum_gx = 0.0, sum_gx_sq = 0.0;
    double sum_gy = 0.0, sum_gy_sq = 0.0;
    double sum_gz = 0.0, sum_gz_sq = 0.0;

    for (uint16_t i = 0; i < N; ++i) {
        const Sample& s = _buffer[i];

        double ax = (double)s.ax;
        double ay = (double)s.ay;
        double az = (double)s.az;
        double gx = (double)s.gx;
        double gy = (double)s.gy;
        double gz = (double)s.gz;

        // Euclidean magnitudes
        double acc_mag = sqrt(ax * ax + ay * ay + az * az);
        double gyro_mag = sqrt(gx * gx + gy * gy + gz * gz);

        if (acc_mag > acc_mag_max) acc_mag_max = acc_mag;
        if (acc_mag < acc_mag_min) acc_mag_min = acc_mag;
        if (gyro_mag > gyro_mag_max) gyro_mag_max = gyro_mag;

        sum_acc_mag += acc_mag;
        sum_acc_mag_sq += acc_mag * acc_mag;

        sum_gyro_mag += gyro_mag;
        sum_gyro_mag_sq += gyro_mag * gyro_mag;

        sum_ax += ax; sum_ax_sq += ax * ax;
        sum_ay += ay; sum_ay_sq += ay * ay;
        sum_az += az; sum_az_sq += az * az;

        sum_gx += gx; sum_gx_sq += gx * gx;
        sum_gy += gy; sum_gy_sq += gy * gy;
        sum_gz += gz; sum_gz_sq += gz * gz;
    }

    double invN = 1.0 / (double)N;

    double acc_mag_mean = sum_acc_mag * invN;
    double acc_mag_var = (sum_acc_mag_sq * invN) - (acc_mag_mean * acc_mag_mean);
    double acc_mag_std = (acc_mag_var > 0.0) ? sqrt(acc_mag_var) : 0.0;
    double acc_mag_p2p = acc_mag_max - acc_mag_min;

    double gyro_mag_mean = sum_gyro_mag * invN;
    double gyro_mag_var = (sum_gyro_mag_sq * invN) - (gyro_mag_mean * gyro_mag_mean);
    double gyro_mag_std = (gyro_mag_var > 0.0) ? sqrt(gyro_mag_var) : 0.0;

    double ax_mean = sum_ax * invN;
    double ax_var = (sum_ax_sq * invN) - (ax_mean * ax_mean);
    double ax_std = (ax_var > 0.0) ? sqrt(ax_var) : 0.0;

    double ay_mean = sum_ay * invN;
    double ay_var = (sum_ay_sq * invN) - (ay_mean * ay_mean);
    double ay_std = (ay_var > 0.0) ? sqrt(ay_var) : 0.0;

    double az_mean = sum_az * invN;
    double az_var = (sum_az_sq * invN) - (az_mean * az_mean);
    double az_std = (az_var > 0.0) ? sqrt(az_var) : 0.0;

    double gx_mean = sum_gx * invN;
    double gx_var = (sum_gx_sq * invN) - (gx_mean * gx_mean);
    double gx_std = (gx_var > 0.0) ? sqrt(gx_var) : 0.0;

    double gy_mean = sum_gy * invN;
    double gy_var = (sum_gy_sq * invN) - (gy_mean * gy_mean);
    double gy_std = (gy_var > 0.0) ? sqrt(gy_var) : 0.0;

    double gz_mean = sum_gz * invN;
    double gz_var = (sum_gz_sq * invN) - (gz_mean * gz_mean);
    double gz_std = (gz_var > 0.0) ? sqrt(gz_var) : 0.0;

    double acc_energy = sum_acc_mag_sq * invN;
    double gyro_energy = sum_gyro_mag_sq * invN;

    // Ordered assignment matching canonical feature names
    featureVector[0]  = acc_mag_max;
    featureVector[1]  = acc_mag_min;
    featureVector[2]  = acc_mag_mean;
    featureVector[3]  = acc_mag_std;
    featureVector[4]  = acc_mag_p2p;
    featureVector[5]  = gyro_mag_max;
    featureVector[6]  = gyro_mag_mean;
    featureVector[7]  = gyro_mag_std;
    featureVector[8]  = ax_std;
    featureVector[9]  = ay_std;
    featureVector[10] = az_std;
    featureVector[11] = gx_std;
    featureVector[12] = gy_std;
    featureVector[13] = gz_std;
    featureVector[14] = acc_energy;
    featureVector[15] = gyro_energy;
}

bool ActivityDetector::detect(ActivityDetectionResult& result) {
    if (!isWindowReady()) {
        return false;
    }

    uint32_t startUs = micros();
    _samplesSinceLastEval = 0;

    double features[MODEL_FEATURE_COUNT];
    computeFeatures(features);

    double scores[MODEL_CLASS_COUNT];
    memset(scores, 0, sizeof(scores));
    evaluate_model(features, scores);

    // Numerically stable Softmax calculation over raw model scores/logits
    double max_val = scores[0];
    for (int i = 1; i < MODEL_CLASS_COUNT; ++i) {
        if (scores[i] > max_val) {
            max_val = scores[i];
        }
    }

    double sum = 0.0;
    double probs[MODEL_CLASS_COUNT];
    for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
        probs[i] = exp(scores[i] - max_val);
        sum += probs[i];
    }

    int bestIdx = 0;
    double bestProb = 0.0;
    for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
        if (sum > 0.0) {
            probs[i] /= sum;
        }
        if (probs[i] > bestProb) {
            bestProb = probs[i];
            bestIdx = i;
        }
    }

    float confidence = (float)bestProb;
    if (confidence > 1.0f) confidence = 1.0f;
    if (confidence < 0.0f) confidence = 0.0f;

    bool isFallClass = MODEL_IS_FALL_CLASS[bestIdx];

    // Additional biomechanical confirmation for fall:
    // Peak acceleration must exceed the minimum physical impact threshold (in m/s^2)
    // features[0] is acc_mag_max in g -> convert to m/s^2
    float peakImpactMs2 = (float)features[0] * GRAVITY_MS2;
    if (isFallClass && peakImpactMs2 < _minFallImpactMs2) {
        // Fall dismissed due to insufficient physical impact peak
        isFallClass = false;
    }

    // Debouncing / Cooldown logic for falls
    uint32_t nowMs = millis();
    if (isFallClass) {
        if (_lastFallTimestampMs != 0 && (nowMs - _lastFallTimestampMs < _fallCooldownMs)) {
            // Cooldown active, suppress repeating emergency fall alert
            isFallClass = false;
        } else {
            _lastFallTimestampMs = nowMs;
        }
    }

    uint32_t elapsedUs = micros() - startUs;

    result.classIndex = bestIdx;
    result.className = MODEL_CLASS_NAMES[bestIdx];
    result.stateCode = MODEL_CLASS_STATE_CODES[bestIdx];
    result.confidence = confidence;
    result.isFall = isFallClass;
    result.inferenceTimeUs = elapsedUs;

    for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
        result.classScores[i] = probs[i];
    }

    return true;
}

