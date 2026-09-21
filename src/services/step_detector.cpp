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
      _timestampHead(0),
      _timestampCount(0) {
    memset(_stepTimestamps, 0, sizeof(_stepTimestamps));
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
    _timestampHead = 0;
    _timestampCount = 0;
    memset(_stepTimestamps, 0, sizeof(_stepTimestamps));
}

void StepDetector::addStepTimestamp(uint32_t timestampMs) {
    _stepTimestamps[_timestampHead] = timestampMs;
    _timestampHead = (_timestampHead + 1) % MAX_CADENCE_HISTORY;
    if (_timestampCount < MAX_CADENCE_HISTORY) {
        _timestampCount++;
    }
}

void StepDetector::cleanCadenceBuffer(uint32_t nowMs) {
    // Drop timestamps older than CADENCE_WINDOW_MS
    while (_timestampCount > 0) {
        size_t oldestIdx = (_timestampHead + MAX_CADENCE_HISTORY - _timestampCount) % MAX_CADENCE_HISTORY;
        if (nowMs - _stepTimestamps[oldestIdx] > CADENCE_WINDOW_MS) {
            _timestampCount--;
        } else {
            break;
        }
    }
}

uint8_t StepDetector::calculateCadence(uint32_t nowMs) {
    if (_lastStepTimeMs == 0 || (nowMs - _lastStepTimeMs > CADENCE_TIMEOUT_MS)) {
        return 0;
    }

    cleanCadenceBuffer(nowMs);

    if (_timestampCount < 2) {
        return 0;
    }

    // Oldest and newest step timestamps within the sliding window
    size_t oldestIdx = (_timestampHead + MAX_CADENCE_HISTORY - _timestampCount) % MAX_CADENCE_HISTORY;
    size_t newestIdx = (_timestampHead + MAX_CADENCE_HISTORY - 1) % MAX_CADENCE_HISTORY;

    uint32_t deltaMs = _stepTimestamps[newestIdx] - _stepTimestamps[oldestIdx];
    if (deltaMs < 200) {
        return 0;
    }

    float dtSec = (float)deltaMs / 1000.0f;
    // Step interval count is (count - 1)
    float spm = (((float)(_timestampCount - 1)) / dtSec) * 60.0f;

    if (spm < 0.0f) spm = 0.0f;
    if (spm > 255.0f) spm = 255.0f;

    return (uint8_t)roundf(spm);
}

bool StepDetector::processSample(float ax, float ay, float az, uint8_t currentActivityState, uint32_t nowMs) {
    // 1. Dynamic vertical acceleration (Earth gravity removed: a_vert = Az - 1.0g)
    float aVert = az - 1.0f;

    // 2. Refractory debounce check
    bool canTrigger = (_lastStepTimeMs == 0) || (nowMs - _lastStepTimeMs >= _refractoryMs);

    bool stepConfirmed = false;

    switch (_state) {
        case StepState::ARMED:
            if (canTrigger && aVert > _stepThreshold) {
                _state = StepState::PEAK_DETECTED;
                _peakVertAccel = aVert;
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

    if (!stepConfirmed) {
        return false;
    }

    // 3. Activity Context Gating (Edge AI)
    // If state is idle or any fall variant, dismiss step
    if (currentActivityState == STATE_CODE_IDLE ||
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
        case STATE_CODE_STAIRS:
            _stairsSteps++;
            break;
        default:
            _unclassifiedSteps++;
            break;
    }

    // Record step timestamp for sliding window cadence estimation
    addStepTimestamp(nowMs);

    return true;
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

    return payload;
}
