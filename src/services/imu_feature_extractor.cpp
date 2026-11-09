#include "imu_feature_extractor.h"

void ImuFeatureExtractor::computeFeatures(
    const ImuSample* buffer,
    uint16_t writeIndex,
    uint32_t sampleCount,
    double* featureVector
) {
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
        const ImuSample& s = buffer[i];

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
#if MODEL_FEATURE_COUNT >= 19
    featureVector[16] = ax_mean;
    featureVector[17] = ay_mean;
    featureVector[18] = az_mean;
#endif
#if MODEL_FEATURE_COUNT >= 22
    // Chronological start and end sample indices in ring buffer
    uint16_t oldestIdx = (sampleCount < MODEL_WINDOW_SAMPLES) ? 0 : writeIndex;
    uint16_t newestIdx = (writeIndex + MODEL_WINDOW_SAMPLES - 1) % MODEL_WINDOW_SAMPLES;
    double ay_start = (double)buffer[oldestIdx].ay;
    double ay_end = (double)buffer[newestIdx].ay;
    double ay_delta_end_start = ay_end - ay_start;

    // Energy distribution: first half vs second half
    double first_half_energy = 0.0;
    double second_half_energy = 0.0;
    uint16_t halfN = N / 2;
    for (uint16_t i = 0; i < N; ++i) {
        uint16_t ringIdx = (oldestIdx + i) % MODEL_WINDOW_SAMPLES;
        const ImuSample& s = buffer[ringIdx];
        double magSq = (double)s.ax * s.ax + (double)s.ay * s.ay + (double)s.az * s.az;
        if (i < halfN) {
            first_half_energy += magSq;
        } else {
            second_half_energy += magSq;
        }
    }
    double acc_energy_half_diff = (first_half_energy - second_half_energy) * invN;

    featureVector[19] = ay_delta_end_start;
    featureVector[20] = acc_energy_half_diff;
    featureVector[21] = gz_mean;
#endif
#if MODEL_FEATURE_COUNT >= 24
    uint16_t tailN = N - halfN;
    double invTailN = (tailN > 0) ? (1.0 / (double)tailN) : 0.0;
    double sum_tail_acc_mag = 0.0;
    double sum_tail_acc_mag_sq = 0.0;
    double sum_tail_gyro_mag = 0.0;
    for (uint16_t i = halfN; i < N; ++i) {
        uint16_t ringIdx = (oldestIdx + i) % MODEL_WINDOW_SAMPLES;
        const ImuSample& s = buffer[ringIdx];
        double a_mag = sqrt((double)s.ax * s.ax + (double)s.ay * s.ay + (double)s.az * s.az);
        double g_mag = sqrt((double)s.gx * s.gx + (double)s.gy * s.gy + (double)s.gz * s.gz);
        sum_tail_acc_mag += a_mag;
        sum_tail_acc_mag_sq += a_mag * a_mag;
        sum_tail_gyro_mag += g_mag;
    }
    double tail_acc_mean = sum_tail_acc_mag * invTailN;
    double tail_acc_var = (sum_tail_acc_mag_sq * invTailN) - (tail_acc_mean * tail_acc_mean);
    double acc_mag_std_tail = (tail_acc_var > 0.0) ? sqrt(tail_acc_var) : 0.0;
    double gyro_mag_mean_tail = sum_tail_gyro_mag * invTailN;

    featureVector[22] = acc_mag_std_tail;
    featureVector[23] = gyro_mag_mean_tail;
#endif
#if MODEL_FEATURE_COUNT >= 26
    // Chronological jerk: max delta between consecutive acceleration magnitudes
    double acc_jerk_max = 0.0;
    double prev_a_mag = 0.0;
    for (uint16_t i = 0; i < N; ++i) {
        uint16_t ringIdx = (oldestIdx + i) % MODEL_WINDOW_SAMPLES;
        const ImuSample& s = buffer[ringIdx];
        double a_mag = sqrt((double)s.ax * s.ax + (double)s.ay * s.ay + (double)s.az * s.az);
        if (i > 0) {
            double delta = fabs(a_mag - prev_a_mag);
            if (delta > acc_jerk_max) {
                acc_jerk_max = delta;
            }
        }
        prev_a_mag = a_mag;
    }
    double gyro_mag_max_to_mean = (gyro_mag_mean > 1e-4) ? (gyro_mag_max / (gyro_mag_mean + 1e-4)) : (gyro_mag_max / 1e-4);

    featureVector[24] = acc_jerk_max;
    featureVector[25] = gyro_mag_max_to_mean;
#endif
#if MODEL_FEATURE_COUNT >= 28
    double ay_min_1 = 1e9, ay_max_1 = -1e9;
    double ay_min_2 = 1e9, ay_max_2 = -1e9;
    for (uint16_t i = 0; i < N; ++i) {
        uint16_t ringIdx = (oldestIdx + i) % MODEL_WINDOW_SAMPLES;
        double ay_val = (double)buffer[ringIdx].ay;
        if (i < halfN) {
            if (ay_val < ay_min_1) ay_min_1 = ay_val;
            if (ay_val > ay_max_1) ay_max_1 = ay_val;
        } else {
            if (ay_val < ay_min_2) ay_min_2 = ay_val;
            if (ay_val > ay_max_2) ay_max_2 = ay_val;
        }
    }
    double ay_p2p_first_half = (halfN > 0) ? (ay_max_1 - ay_min_1) : 0.0;
    double ay_p2p_second_half = (tailN > 0) ? (ay_max_2 - ay_min_2) : 0.0;

    featureVector[26] = ay_p2p_first_half;
    featureVector[27] = ay_p2p_second_half;
#endif
}
