#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct quat
{
    float q_[4];
} mathQuaternion_t;

enum
{
    MATH_QUATERNION_W = 0,
    MATH_QUATERNION_X,
    MATH_QUATERNION_Y,
    MATH_QUATERNION_Z,
};

// 原地归一化
bool mathQuaternionNormalizeInPlace(mathQuaternion_t *quat);
// 归一化
bool mathQuaternionNormalize(const mathQuaternion_t *quat, mathQuaternion_t *quat_out);
bool mathQuaternionNormSquare(const mathQuaternion_t *quat, float *out);
bool mathQuaternionNorm(const mathQuaternion_t *quat, float *out);
bool mathQuaternionSetIdentity(mathQuaternion_t *quat);
// 共轭四元数
bool mathQuaternionConjugate(const mathQuaternion_t *quat, mathQuaternion_t *quat_out);
// 四元数乘法，请注意顺序
bool mathQuaternionMultiply(const mathQuaternion_t *quat_1, const mathQuaternion_t *quat_2, mathQuaternion_t *quat_out);
bool mathQuaternionBuildFromSmallAngleError(const float angle_error_rad[3], mathQuaternion_t *quat_out);
bool mathQuaternionBuildFromSmallAngleErrorLinear(const float angle_error_rad[3], mathQuaternion_t *quat_out);
bool mathQuaternionUpdateBySmallAngleErrorInPlace(const float angle_error_rad[3], mathQuaternion_t *quat);
bool mathQuaternionUpdateBySmallAngleError(const float angle_error_rad[3], const mathQuaternion_t *quat, mathQuaternion_t *quat_out);
bool mathQuaternionToRotationMatrix(const mathQuaternion_t *quat, float matrix_out[9]);
bool mathQuaternionRotateVector3(const mathQuaternion_t *quat, const float vec3[3], float vec3_out[3]);
bool mathQuaternionToEulerZYX(const mathQuaternion_t *quat, float euler_zyx_rad_out[3]);
