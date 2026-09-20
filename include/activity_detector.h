#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "invariants.h"
#include "activity_model_generated.h"

/**
 * @file activity_detector.h
 * @brief Real-time on-device biomechanical feature extraction and activity inference engine for ESP32-S3.
 */

struct ActivityDetectionResult {
    int classIndex;             // Index in MODEL_CLASS_NAMES (e.g., 0 to MODEL_CLASS_COUNT - 1)
    const char* className;      // Human-readable class name (e.g., "walk", "idle", "fall_forward")
    uint8_t stateCode;          // BLE state code (e.g., STATE_CODE_WALK, STATE_CODE_FALL_FORWARD)
    float confidence;           // Softmax/voting confidence score (0.0f to 1.0f)
    bool isFall;                // True if predicted activity is classified as a fall
    uint32_t inferenceTimeUs;   // Time spent computing features and running model in microseconds
    double classScores[MODEL_CLASS_COUNT]; // Individual raw prediction scores for all classes
};

class ActivityDetector {
public:
    ActivityDetector();

    /**
     * @brief Initializes the sliding window buffer and inference parameters.
     * @param confidenceThreshold Minimum probability required to confirm an activity (default: 0.65f).
     * @param fallCooldownMs Minimum interval in ms between consecutive fall alerts (default: 5000 ms).
     * @param minFallImpactMs2 Minimum peak acceleration in m/s^2 to confirm impact (default: 18.0 m/s^2).
     */
    void begin(
        float confidenceThreshold = 0.65f,
        uint32_t fallCooldownMs = 5000,
        float minFallImpactMs2 = 18.0f
    );

    /**
     * @brief Pushes a new IMU sample into the sliding ring buffer.
     * @param ax Acceleration X in g.
     * @param ay Acceleration Y in g.
     * @param az Acceleration Z in g.
     * @param gx Angular velocity X in deg/s.
     * @param gy Angular velocity Y in deg/s.
     * @param gz Angular velocity Z in deg/s.
     */
    void pushSample(float ax, float ay, float az, float gx, float gy, float gz);

    /**
     * @brief Pushes a compressed 14-byte ImuRawFrame into the sliding ring buffer.
     * @param frame Big-endian network frame from imu.readFrame().
     */
    void pushFrame(const ImuRawFrame& frame);

    /**
     * @brief Returns true if a full window of IMU samples has been collected.
     */
    bool isWindowReady() const { return _sampleCount >= MODEL_WINDOW_SAMPLES; }

    /**
     * @brief Returns true if a new step interval (overlap) has elapsed since the last evaluation.
     */
    bool isEvaluationDue() const;

    /**
     * @brief Extracts 16 biomechanical features from current window and evaluates the classifier.
     * @param[out] result Output detection result with class, state code, confidence and metrics.
     * @return true if classification completed successfully, false if insufficient samples.
     */
    bool detect(ActivityDetectionResult& result);

    /**
     * @brief Resets the sliding window buffer (e.g. when transitioning from Studio recording mode).
     */
    void reset();

    // Getters / Configuration
    void setConfidenceThreshold(float threshold) { _confidenceThreshold = threshold; }
    float getConfidenceThreshold() const { return _confidenceThreshold; }
    void setFallCooldown(uint32_t cooldownMs) { _fallCooldownMs = cooldownMs; }
    uint32_t getSampleCount() const { return _sampleCount; }

private:
    struct Sample {
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
    };

    Sample _buffer[MODEL_WINDOW_SAMPLES];
    uint16_t _writeIndex;
    uint32_t _sampleCount;
    uint32_t _samplesSinceLastEval;

    float _confidenceThreshold;
    uint32_t _fallCooldownMs;
    float _minFallImpactMs2;
    uint32_t _lastFallTimestampMs;

    // Feature extraction routine
    void computeFeatures(double* featureVector);
};

