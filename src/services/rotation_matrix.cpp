#include "rotation_matrix.h"
#include <math.h>

void RotationMatrix3D::setIdentity(float r[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

bool RotationMatrix3D::fromGravityVector(float vx, float vy, float vz, float r[3][3],
                                        float* outNx, float* outNy, float* outNz) {
    double mag = sqrt(vx * vx + vy * vy + vz * vz);
    if (mag < 1e-4) {
        return false;
    }

    // Normalized measured gravity vector
    float nx = (float)(vx / mag);
    float ny = (float)(vy / mag);
    float nz = (float)(vz / mag);

    if (outNx != nullptr) *outNx = nx;
    if (outNy != nullptr) *outNy = ny;
    if (outNz != nullptr) *outNz = nz;

    // Target vertical gravity vector in shoe body frame: v_target = [0, 0, 1.0]
    // Rotation axis: u = v_meas x v_target = [ny, -nx, 0]
    // Cosine of angle: c = v_meas . v_target = nz
    float s = sqrtf(nx * nx + ny * ny);

    if (s < 1e-5f || (1.0f + nz) < 1e-5f) {
        // Already aligned or antiparallel safety fallback
        setIdentity(r);
    } else {
        // Rodrigues' rotation formula: R = I + [u]_x + [u]_x^2 / (1 + c)
        float k = 1.0f / (1.0f + nz);

        r[0][0] = 1.0f - k * nx * nx;
        r[0][1] = -k * nx * ny;
        r[0][2] = -nx;

        r[1][0] = -k * nx * ny;
        r[1][1] = 1.0f - k * ny * ny;
        r[1][2] = -ny;

        r[2][0] = nx;
        r[2][1] = ny;
        r[2][2] = nz;
    }

    return true;
}

void RotationMatrix3D::rotateVector(const float r[3][3], float& x, float& y, float& z) {
    float rx = r[0][0] * x + r[0][1] * y + r[0][2] * z;
    float ry = r[1][0] * x + r[1][1] * y + r[1][2] * z;
    float rz = r[2][0] * x + r[2][1] * y + r[2][2] * z;
    x = rx;
    y = ry;
    z = rz;
}

void RotationMatrix3D::flatten(const float r[3][3], float flat[9]) {
    if (flat == nullptr) return;
    int idx = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            flat[idx++] = r[i][j];
        }
    }
}

void RotationMatrix3D::fromFlat(const float flat[9], float r[3][3]) {
    if (flat == nullptr) return;
    int idx = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r[i][j] = flat[idx++];
        }
    }
}
