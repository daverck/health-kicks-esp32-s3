#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @file rotation_matrix.h
 * @brief 3D vector rotation and Rodrigues' rotation matrix utilities for IMU calibration.
 */
class RotationMatrix3D {
public:
    /**
     * @brief Sets a 3x3 rotation matrix to the identity matrix.
     */
    static void setIdentity(float r[3][3]);

    /**
     * @brief Computes a 3x3 rotation matrix aligning a measured gravity vector to vertical [0, 0, 1].
     * 
     * Uses Rodrigues' rotation formula:
     *   R = I + [u]_x + [u]_x^2 / (1 + c)
     * where u = v_meas x v_target = [ny, -nx, 0] and c = v_meas . v_target = nz.
     * 
     * @param vx Measured gravity component X.
     * @param vy Measured gravity component Y.
     * @param vz Measured gravity component Z.
     * @param[out] r Resulting 3x3 rotation matrix.
     * @param[out] outNx Optional pointer to receive normalized nx.
     * @param[out] outNy Optional pointer to receive normalized ny.
     * @param[out] outNz Optional pointer to receive normalized nz.
     * @return true if computation succeeded, false if magnitude is degenerate (< 1e-4).
     */
    static bool fromGravityVector(float vx, float vy, float vz, float r[3][3],
                                 float* outNx = nullptr, float* outNy = nullptr, float* outNz = nullptr);

    /**
     * @brief Rotates a 3D vector using the given 3x3 rotation matrix: v' = R * v.
     * 
     * @param r 3x3 rotation matrix.
     * @param[in,out] x Vector X component.
     * @param[in,out] y Vector Y component.
     * @param[in,out] z Vector Z component.
     */
    static void rotateVector(const float r[3][3], float& x, float& y, float& z);

    /**
     * @brief Flattens a 3x3 matrix into a 9-element array (row-major).
     */
    static void flatten(const float r[3][3], float flat[9]);

    /**
     * @brief Restores a 3x3 matrix from a 9-element array (row-major).
     */
    static void fromFlat(const float flat[9], float r[3][3]);
};
