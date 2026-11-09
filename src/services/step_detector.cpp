#include "step_detector.h"

StepDetector::StepDetector()
    : _stepThreshold(DEFAULT_STEP_THRESHOLD_G),
      _refractoryMs(DEFAULT_REFRACTORY_MS),
      _state(StepState::ARMED),
      _totalSteps(0),
      _walkSteps(0),
      _runSteps(0),
      _stairsSteps(0),
      _unclassifiedSteps(0),
      _lastStepTimeMs(0),
      _peakVertAccel(0.0f),
      _stanceStartMs(0),
      _inStance(false),
      _maxPitchRateInStance(0.0f),
      _smoothGctMs(0.0f),
      _lastGctMs(0),
      _cadenceTracker(CADENCE_WINDOW_MS, CADENCE_TIMEOUT_MS) {
}

void StepDetector::begin(float stepThreshold, uint32_t refractoryMs) {
    _stepThreshold = stepThreshold > 0.05f ? stepThreshold : DEFAULT_STEP_THRESHOLD_G;
    _refractoryMs = refractoryMs > 50 ? refractoryMs : DEFAULT_REFRACTORY_MS;
    reset();
}

void StepDetector::reset() {
    _state = StepState::ARMED;
    _totalSteps = 0;
    _walkSteps = 0;
    _runSteps = 0;
    _stairsSteps = 0;
    _unclassifiedSteps = 0;
    _lastStepTimeMs = 0;
    _peakVertAccel = 0.0f;
    _stanceStartMs = 0;
    _inStance = false;
    _maxPitchRateInStance = 0.0f;
    _smoothGctMs = 0.0f;
    _lastGctMs = 0;
    _cadenceTracker.reset();
}

void StepDetector::restoreState(uint32_t total, uint16_t walk, uint16_t run, uint16_t stairs, uint16_t unclassified) {
    _totalSteps = total;
    _walkSteps = walk;
    _runSteps = run;
    _stairsSteps = stairs;
    _unclassifiedSteps = unclassified;
}

uint8_t StepDetector::calculateCadence(uint32_t nowMs) {
    return _cadenceTracker.calculateCadence(nowMs, _lastStepTimeMs);
}

bool StepDetector::processSample(float ax, float ay, float az, float gy, uint8_t currentActivityState, uint32_t nowMs) {
    // 1. Dynamic vertical acceleration (Earth gravity removed: a_vert = Az - 1.0g)
    float aVert = az - 1.0f;
    float absGy = fabsf(gy);

    // 2. Refractory debounce check
    bool canTrigger = (_lastStepTimeMs == 0) || (nowMs - _lastStepTimeMs >= _refractoryMs);

    bool stepConfirmed = false;

    // Step heel-strike detection state machine
    switch (_state) {
        case StepState::ARMED:
            if (canTrigger && aVert > _stepThreshold) {
                // Rising edge: initial heel contact impact (Az > 1.35g)
                _state = StepState::PEAK_DETECTED;
                _peakVertAccel = aVert;
                _stanceStartMs = nowMs;
                _inStance = true;
                _maxPitchRateInStance = absGy;
            }
            break;

        case StepState::PEAK_DETECTED:
            if (aVert > _peakVertAccel) {
                _peakVertAccel = aVert;
            } else if (aVert < (_stepThreshold * 0.5f)) {
                // Crossing back below threshold confirms heel-strike event
                _state = StepState::ARMED;
                _lastStepTimeMs = nowMs;
                stepConfirmed = true;
            }
            break;
    }

    // Ground Contact Time (GCT) propulsion & toe-off tracking
    if (_inStance) {
        uint32_t elapsedMs = nowMs - _stanceStartMs;

        // Monitor pitch angular velocity during 140 to 900 ms following impact
        if (elapsedMs >= 140 && elapsedMs <= 900) {
            if (absGy > _maxPitchRateInStance) {
                _maxPitchRateInStance = absGy;
            } else if ((_maxPitchRateInStance > 25.0f && absGy < (_maxPitchRateInStance * 0.8f) && aVert < 0.25f) ||
                       (elapsedMs >= 350 && aVert < 0.05f)) {
                // Propulsion peak crossed and vertical acceleration dropped: toe-off event detected!
                _inStance = false;
                updateGct(elapsedMs, currentActivityState);
            }
        } else if (elapsedMs > 900) {
            // Stance timeout without clean toe-off
            _inStance = false;
        }
    }

    if (!stepConfirmed) {
        return false;
    }

    // 3. Activity Context Gating (Edge AI)
    // If state is idle, seated, or any fall variant, dismiss step
    if (currentActivityState == STATE_CODE_IDLE ||
        currentActivityState == STATE_CODE_SIT_DOWN ||
        currentActivityState == STATE_CODE_FALL_FORWARD ||
        currentActivityState == STATE_CODE_FALL_BACKWARD ||
        currentActivityState == STATE_CODE_FALL_LATERAL ||
        currentActivityState == STATE_CODE_FALL_GENERIC) {
        return false;
    }

    // Count step and attribute according to active gait classification
    _totalSteps++;

    switch (currentActivityState) {
        case STATE_CODE_WALK:
            _walkSteps++;
            break;
        case STATE_CODE_RUN:
            _runSteps++;
            break;
        case STATE_CODE_STAIRS_UP:
        case STATE_CODE_STAIRS_DOWN:
            _stairsSteps++;
            break;
        default:
            _unclassifiedSteps++;
            break;
    }

    // Record step timestamp for sliding window cadence estimation
    _cadenceTracker.addStepTimestamp(nowMs);

    return true;
}

void StepDetector::updateGct(uint32_t instantGct, uint8_t activityState) {
    bool isPlausible = false;

    if (activityState == STATE_CODE_RUN) {
        // Running GCT plausible between 140 ms and 380 ms
        isPlausible = (instantGct >= 140 && instantGct <= 380);
    } else if (activityState == STATE_CODE_WALK) {
        // Walking GCT plausible between 350 ms and 900 ms
        isPlausible = (instantGct >= 350 && instantGct <= 900);
    } else {
        // General plausible window for stairs and other activities
        isPlausible = (instantGct >= 140 && instantGct <= 900);
    }

    if (!isPlausible) {
        return;
    }

    // Exponential Moving Average (EMA) over ~5 steps:
    // GCT_smooth = 0.7 * GCT_smooth + 0.3 * GCT_instant
    if (_smoothGctMs <= 0.0f) {
        _smoothGctMs = (float)instantGct;
    } else {
        _smoothGctMs = 0.7f * _smoothGctMs + 0.3f * (float)instantGct;
    }

    _lastGctMs = (uint16_t)roundf(_smoothGctMs);
}

StepCounterPayload StepDetector::getPayload(uint32_t nowMs) {
    StepCounterPayload payload;

    // Convert to Network Big-Endian
    payload.total_steps = ((_totalSteps >> 24) & 0xFF) |
                          ((_totalSteps >> 8) & 0xFF00) |
                          ((_totalSteps << 8) & 0xFF0000) |
                          ((_totalSteps << 24) & 0xFF000000);

    payload.walk_steps = (uint16_t)(((_walkSteps >> 8) & 0x00FF) | ((_walkSteps << 8) & 0xFF00));
    payload.run_steps = (uint16_t)(((_runSteps >> 8) & 0x00FF) | ((_runSteps << 8) & 0xFF00));
    payload.stairs_steps = (uint16_t)(((_stairsSteps >> 8) & 0x00FF) | ((_stairsSteps << 8) & 0xFF00));
    payload.unclassified_steps = (uint16_t)(((_unclassifiedSteps >> 8) & 0x00FF) | ((_unclassifiedSteps << 8) & 0xFF00));
    payload.cadence_spm = calculateCadence(nowMs);

    uint16_t gct = getAverageGctMs();
    payload.gct_ms = (uint16_t)(((gct >> 8) & 0x00FF) | ((gct << 8) & 0xFF00));

    return payload;
}

