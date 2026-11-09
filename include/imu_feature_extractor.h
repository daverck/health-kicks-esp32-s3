#pragma once

#include <stdint.h>
#include <math.h>
#include "activity_model_generated.h"

/**
 * @struct ImuSample
 * @brief 6-DOF IMU sample containing acceleration (in g) and angular velocity (in deg/s).
 */
struct ImuSample {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

/**
 * @class ImuFeatureExtractor
 * @brief Biomechanical signal processing module computing time-domain and frequency/energy features from IMU sample windows.
 */
class ImuFeatureExtractor {
public:
    /**
     * @brief Computes MODEL_FEATURE_COUNT biomechanical features from an IMU sample ring buffer.
     * @param buffer Array of MODEL_WINDOW_SAMPLES samples.
     * @param writeIndex Current ring buffer write index.
     * @param sampleCount Total samples pushed into buffer.
     * @param[out] featureVector Output array receiving computed features.
     */
    static void computeFeatures(
        const ImuSample* buffer,
        uint16_t writeIndex,
        uint32_t sampleCount,
        double* featureVector
    );
};
