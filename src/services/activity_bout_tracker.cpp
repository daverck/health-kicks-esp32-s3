#include "activity_bout_tracker.h"
#include <string.h>

#ifndef htons
#define htons(x) ((uint16_t)((((uint16_t)(x) & 0x00FFU) << 8) | (((uint16_t)(x) & 0xFF00U) >> 8)))
#endif

ActivityBoutTracker::ActivityBoutTracker()
    : _head(0),
      _tail(0),
      _count(0),
      _activeBoutOpen(false),
      _activeStartUptimeSec(0),
      _activeLastStepUptimeSec(0),
      _activeStepCount(0),
      _activeActivityType(0x04),
      _activeCadenceSum(0),
      _activeCadenceSamples(0) {
    memset(_bouts, 0, sizeof(_bouts));
}

void ActivityBoutTracker::begin() {
    reset();
}

uint8_t ActivityBoutTracker::normalizeActivityType(uint8_t stateCode) {
    switch (stateCode) {
        case STATE_CODE_WALK:
            return 0x01;
        case STATE_CODE_RUN:
            return 0x02;
        case STATE_CODE_STAIRS_UP:
        case STATE_CODE_STAIRS_DOWN:
            return 0x03;
        default:
            return 0x04; // Unclassified
    }
}

void ActivityBoutTracker::onStep(uint8_t activityState, uint32_t nowMs, uint8_t cadence) {
    uint32_t uptimeSec = nowMs / 1000;
    uint8_t normalizedType = normalizeActivityType(activityState);

    if (!_activeBoutOpen) {
        _activeBoutOpen = true;
        _activeStartUptimeSec = uptimeSec;
        _activeLastStepUptimeSec = uptimeSec;
        _activeStepCount = 1;
        _activeActivityType = normalizedType;
        _activeCadenceSum = cadence;
        _activeCadenceSamples = (cadence > 0) ? 1 : 0;
        return;
    }

    // If activity category changed significantly (e.g. from Walk to Run or vice versa)
    // after at least 2 steps, finalize the previous bout and start a new one.
    if (normalizedType != _activeActivityType && _activeStepCount >= 2) {
        finalizeCurrentBout(_activeLastStepUptimeSec);
        _activeBoutOpen = true;
        _activeStartUptimeSec = uptimeSec;
        _activeLastStepUptimeSec = uptimeSec;
        _activeStepCount = 1;
        _activeActivityType = normalizedType;
        _activeCadenceSum = cadence;
        _activeCadenceSamples = (cadence > 0) ? 1 : 0;
        return;
    }

    _activeStepCount++;
    _activeLastStepUptimeSec = uptimeSec;
    if (cadence > 0) {
        _activeCadenceSum += cadence;
        _activeCadenceSamples++;
    }
}

void ActivityBoutTracker::onTick(uint32_t nowMs) {
    if (!_activeBoutOpen) return;
    uint32_t uptimeSec = nowMs / 1000;
    if (uptimeSec >= _activeLastStepUptimeSec && (uptimeSec - _activeLastStepUptimeSec) >= STILLNESS_TIMEOUT_SEC) {
        finalizeCurrentBout(_activeLastStepUptimeSec);
    }
}

void ActivityBoutTracker::flushActiveBout(uint32_t nowMs) {
    if (_activeBoutOpen && _activeStepCount > 0) {
        finalizeCurrentBout(nowMs / 1000);
    }
}

void ActivityBoutTracker::finalizeCurrentBout(uint32_t endUptimeSec) {
    if (!_activeBoutOpen) return;
    if (_activeStepCount == 0) {
        _activeBoutOpen = false;
        return;
    }

    uint16_t durationSec = 1;
    if (endUptimeSec > _activeStartUptimeSec) {
        durationSec = (uint16_t)(endUptimeSec - _activeStartUptimeSec);
    }

    uint8_t avgCadence = 0;
    if (_activeCadenceSamples > 0) {
        avgCadence = (uint8_t)(_activeCadenceSum / _activeCadenceSamples);
    } else if (durationSec > 0) {
        uint32_t calc = ((uint32_t)_activeStepCount * 60) / durationSec;
        avgCadence = (calc > 255) ? 255 : (uint8_t)calc;
    }

    ActivityBout bout;
    bout.start_uptime_sec = htons((uint16_t)(_activeStartUptimeSec & 0xFFFF));
    bout.duration_sec = htons(durationSec);
    bout.step_count = htons(_activeStepCount);
    bout.activity_type = _activeActivityType;
    bout.avg_cadence = avgCadence;

    _bouts[_head] = bout;
    _head = (_head + 1) % MAX_BOUTS;
    if (_count < MAX_BOUTS) {
        _count++;
    } else {
        // Ring buffer full: advance tail to discard oldest unacknowledged bout
        _tail = (_tail + 1) % MAX_BOUTS;
    }

    _activeBoutOpen = false;
    _activeStepCount = 0;
    _activeCadenceSum = 0;
    _activeCadenceSamples = 0;
}

void ActivityBoutTracker::getBouts(size_t maxBouts, std::vector<ActivityBout>& outBouts) {
    outBouts.clear();
    size_t numToFetch = (_count < maxBouts) ? _count : maxBouts;
    outBouts.reserve(numToFetch);
    for (size_t i = 0; i < numToFetch; ++i) {
        size_t idx = (_tail + i) % MAX_BOUTS;
        outBouts.push_back(_bouts[idx]);
    }
}

void ActivityBoutTracker::acknowledgeCount(uint8_t count) {
    if (count == 0) return;
    if (count >= _count) {
        _tail = _head;
        _count = 0;
    } else {
        _tail = (_tail + count) % MAX_BOUTS;
        _count -= count;
    }
}

void ActivityBoutTracker::reset() {
    _head = 0;
    _tail = 0;
    _count = 0;
    _activeBoutOpen = false;
    _activeStartUptimeSec = 0;
    _activeLastStepUptimeSec = 0;
    _activeStepCount = 0;
    _activeActivityType = 0x04;
    _activeCadenceSum = 0;
    _activeCadenceSamples = 0;
    memset(_bouts, 0, sizeof(_bouts));
}
