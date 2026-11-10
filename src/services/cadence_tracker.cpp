#include "cadence_tracker.h"
#include <string.h>

CadenceTracker::CadenceTracker(uint32_t windowMs, uint32_t timeoutMs)
    : _windowMs(windowMs)
    , _timeoutMs(timeoutMs)
    , _timestampHead(0)
    , _timestampCount(0)
{
    memset(_stepTimestamps, 0, sizeof(_stepTimestamps));
}

void CadenceTracker::reset() {
    _timestampHead = 0;
    _timestampCount = 0;
    memset(_stepTimestamps, 0, sizeof(_stepTimestamps));
}

void CadenceTracker::addStepTimestamp(uint32_t timestampMs) {
    _stepTimestamps[_timestampHead] = timestampMs;
    _timestampHead = (_timestampHead + 1) % MAX_CADENCE_HISTORY;
    if (_timestampCount < MAX_CADENCE_HISTORY) {
        _timestampCount++;
    }
}

size_t CadenceTracker::getOldestIndex() const {
    return (_timestampHead + MAX_CADENCE_HISTORY - _timestampCount) % MAX_CADENCE_HISTORY;
}

size_t CadenceTracker::getNewestIndex() const {
    return (_timestampHead + MAX_CADENCE_HISTORY - 1) % MAX_CADENCE_HISTORY;
}

uint8_t CadenceTracker::computeStepsPerMinute(size_t intervalCount, uint32_t deltaMs) {
    if (deltaMs < 200 || intervalCount == 0) {
        return 0;
    }

    float dtSec = (float)deltaMs / 1000.0f;
    float spm = (((float)intervalCount) / dtSec) * 60.0f;

    if (spm < 0.0f) spm = 0.0f;
    if (spm > 255.0f) spm = 255.0f;

    return (uint8_t)roundf(spm);
}

void CadenceTracker::cleanCadenceBuffer(uint32_t nowMs) {
    // Drop timestamps older than sliding window
    while (_timestampCount > 0) {
        size_t oldestIdx = getOldestIndex();
        if (nowMs - _stepTimestamps[oldestIdx] > _windowMs) {
            _timestampCount--;
        } else {
            break;
        }
    }
}

uint8_t CadenceTracker::calculateCadence(uint32_t nowMs, uint32_t lastStepTimeMs) {
    if (lastStepTimeMs == 0 || (nowMs - lastStepTimeMs > _timeoutMs)) {
        return 0;
    }

    cleanCadenceBuffer(nowMs);

    if (_timestampCount < 2) {
        return 0;
    }

    // Oldest and newest step timestamps within the sliding window
    size_t oldestIdx = getOldestIndex();
    size_t newestIdx = getNewestIndex();

    uint32_t deltaMs = _stepTimestamps[newestIdx] - _stepTimestamps[oldestIdx];
    return computeStepsPerMinute(_timestampCount - 1, deltaMs);
}
