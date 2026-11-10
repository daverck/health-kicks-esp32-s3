#include "activity_detector.h"
#include "imu_frame_encoder.h"

// Earth gravity constant for converting acceleration between g and m/s^2
static const float GRAVITY_MS2 = 9.80665f;

static void normalizeClassProbabilities(
    const double scores[MODEL_CLASS_COUNT],
    double probs[MODEL_CLASS_COUNT],
    int& outBestIdx,
    double& outBestProb
) {
    // Sum scores to check if model outputs normalized probabilities (RandomForest/ExtraTrees sum to 1.0)
    // or unnormalized logits (MLP/Linear models)
    double sum = 0.0;
    for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
        sum += scores[i];
    }

    outBestIdx = 0;
    outBestProb = 0.0;

    if (sum >= 0.85 && sum <= 1.15) {
        // Model directly outputs normalized class probabilities from tree voting (RandomForest)
        for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
            probs[i] = scores[i] / sum;
            if (probs[i] > outBestProb) {
                outBestProb = probs[i];
                outBestIdx = i;
            }
        }
    } else {
        // Unbounded logits: compute numerically stable Softmax
        double max_val = scores[0];
        for (int i = 1; i < MODEL_CLASS_COUNT; ++i) {
            if (scores[i] > max_val) {
                max_val = scores[i];
            }
        }
        double exp_sum = 0.0;
        for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
            probs[i] = exp(scores[i] - max_val);
            exp_sum += probs[i];
        }
        for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
            if (exp_sum > 0.0) {
                probs[i] /= exp_sum;
            }
            if (probs[i] > outBestProb) {
                outBestProb = probs[i];
                outBestIdx = i;
            }
        }
    }
}

ActivityDetector::ActivityDetector()
    : _writeIndex(0),
      _sampleCount(0),
      _samplesSinceLastEval(0),
      _eventTriggerArmed(false),
      _eventEvaluationDue(false),
      _postEventSamplesCountdown(0),
      _eventRefractoryCountdown(0),
      _prevAccMag(1.0f),
      _confidenceThreshold(0.50f),
      _fallConfidenceThreshold(0.65f),
      _fallCooldownMs(3000),
      _lastFallTimestampMs(0) {
    memset(_buffer, 0, sizeof(_buffer));
}

void ActivityDetector::begin(float confidenceThreshold, uint32_t fallCooldownMs, float fallConfidenceThreshold) {
    _confidenceThreshold = confidenceThreshold;
    _fallCooldownMs = fallCooldownMs;
    _fallConfidenceThreshold = fallConfidenceThreshold;
    reset();
}

void ActivityDetector::reset() {
    _writeIndex = 0;
    _sampleCount = 0;
    _samplesSinceLastEval = 0;
    _eventTriggerArmed = false;
    _eventEvaluationDue = false;
    _postEventSamplesCountdown = 0;
    _eventRefractoryCountdown = 0;
    _prevAccMag = 1.0f;
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

    // Dynamic shock & jerk detection for Event-Centric temporal alignment
    float a_mag = sqrtf(ax * ax + ay * ay + az * az);
    float jerk = fabsf(a_mag - _prevAccMag);
    float g_mag = sqrtf(gx * gx + gy * gy + gz * gz);
    _prevAccMag = a_mag;

    if (_eventRefractoryCountdown > 0) {
        _eventRefractoryCountdown--;
    }

    // Dynamic trigger conditions (relaxed to capture smooth posture transitions as well as impacts):
    // 1. Acceleration deviation from 1.0g >= 0.08g (||a|| >= 1.08g or <= 0.92g unloading)
    // 2. Jerk delta >= 0.15g (~1.5 m/s² per sample at 19 Hz)
    // 3. Angular velocity magnitude >= 35 deg/s
    const bool isDynamicShock = (fabsf(a_mag - 1.0f) >= 0.08f || jerk >= 0.15f || g_mag >= 35.0f);

    if (_sampleCount >= MODEL_WINDOW_SAMPLES) {
        if (isDynamicShock && _eventRefractoryCountdown == 0) {
            // (Re)arm event-centric trigger: collect ~1.05s (20 samples at 19 Hz) post-shock
            _eventTriggerArmed = true;
            _postEventSamplesCountdown = 20;
        }

        if (_eventTriggerArmed) {
            if (_postEventSamplesCountdown > 0) {
                _postEventSamplesCountdown--;
            }
            if (_postEventSamplesCountdown == 0) {
                _eventTriggerArmed = false;
                _eventEvaluationDue = true;
                _eventRefractoryCountdown = 25; // ~1.3s refractory period to debounce consecutive triggers
            }
        }
    }
}

void ActivityDetector::pushFrame(const ImuRawFrame& frame) {
    float ax, ay, az, gx, gy, gz;
    decodeImuRawFrame(frame, ax, ay, az, gx, gy, gz);
    pushSample(ax, ay, az, gx, gy, gz);
}

bool ActivityDetector::isEvaluationDue() const {
    if (_sampleCount < MODEL_WINDOW_SAMPLES) {
        return false;
    }
    // Event-centric trigger (evaluates window centered on shock peak)
    if (_eventEvaluationDue) {
        return true;
    }
    // Fallback: 10 samples (~0.53s, matching 0.5s training step) overlap step for steady-state continuous locomotion
    const uint32_t stepSamples = 10;
    return _samplesSinceLastEval >= stepSamples;
}

void ActivityDetector::computeFeatures(double* featureVector) {
    ImuFeatureExtractor::computeFeatures(_buffer, _writeIndex, _sampleCount, featureVector);
}

bool ActivityDetector::detect(ActivityDetectionResult& result) {
    if (!isWindowReady()) {
        return false;
    }

    uint32_t startUs = micros();
    _samplesSinceLastEval = 0;
    _eventEvaluationDue = false;

    double features[MODEL_FEATURE_COUNT];
    computeFeatures(features);

    double scores[MODEL_CLASS_COUNT];
    memset(scores, 0, sizeof(scores));
    evaluate_model(features, scores);

    double probs[MODEL_CLASS_COUNT];
    int bestIdx = 0;
    double bestProb = 0.0;
    normalizeClassProbabilities(scores, probs, bestIdx, bestProb);

    float confidence = (float)bestProb;
    if (confidence > 1.0f) confidence = 1.0f;
    if (confidence < 0.0f) confidence = 0.0f;

    uint32_t nowMs = millis();

    // Contextual gating removed upon user request: trust pure ML classification for all states.

    bool isFallClass = MODEL_IS_FALL_CLASS[bestIdx];

    // Peak acceleration magnitude in m/s^2 observed in window
    // features[0] is acc_mag_max in g -> convert to m/s^2
    float peakImpactMs2 = (float)features[0] * GRAVITY_MS2;

    if (isFallClass) {
        // Check if confidence meets the fall confidence threshold
        if (confidence < _fallConfidenceThreshold) {
            return false;
        }

        // Debouncing / Cooldown logic for falls (prevents repeat burst notifications)
        if (_lastFallTimestampMs != 0 && (nowMs - _lastFallTimestampMs < _fallCooldownMs)) {
            return false;
        }

        // Physical impact threshold veto removed upon user request to evaluate pure ML performance.
        _lastFallTimestampMs = nowMs;
    }

    uint32_t elapsedUs = micros() - startUs;

    result.classIndex = bestIdx;
    result.className = MODEL_CLASS_NAMES[bestIdx];
    result.stateCode = MODEL_CLASS_STATE_CODES[bestIdx];
    result.confidence = confidence;
    result.isFall = isFallClass;
    result.peakImpactMs2 = peakImpactMs2;
    result.inferenceTimeUs = elapsedUs;

    for (int i = 0; i < MODEL_CLASS_COUNT; ++i) {
        result.classScores[i] = probs[i];
    }

    return true;
}
