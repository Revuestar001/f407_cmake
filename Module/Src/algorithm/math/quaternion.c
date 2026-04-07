#include "arm_math.h"

#include <stdbool.h>

#include "quaternion.h"

#define MATH_QUATERNION_EPSILON 1e-6f

static float mathQuaternionDot(const mathQuaternion_t *quat_1, const mathQuaternion_t *quat_2)
{
    return quat_1->q_[MATH_QUATERNION_W] * quat_2->q_[MATH_QUATERNION_W] +
           quat_1->q_[MATH_QUATERNION_X] * quat_2->q_[MATH_QUATERNION_X] +
           quat_1->q_[MATH_QUATERNION_Y] * quat_2->q_[MATH_QUATERNION_Y] +
           quat_1->q_[MATH_QUATERNION_Z] * quat_2->q_[MATH_QUATERNION_Z];
}

// 原地归一化
bool mathQuaternionNormalizeInPlace(mathQuaternion_t *quat)
{
    if (quat == NULL) {
        return false;
    }

    float quat_norm_square;
    float quat_norm;

    quat_norm_square = mathQuaternionDot(quat, quat);
    if (quat_norm_square <= MATH_QUATERNION_EPSILON * MATH_QUATERNION_EPSILON) {
        return false;
    }

    if (arm_sqrt_f32(quat_norm_square, &quat_norm) != ARM_MATH_SUCCESS) {
        return false;
    }

    quat->q_[MATH_QUATERNION_W] /= quat_norm;
    quat->q_[MATH_QUATERNION_X] /= quat_norm;
    quat->q_[MATH_QUATERNION_Y] /= quat_norm;
    quat->q_[MATH_QUATERNION_Z] /= quat_norm;

    return true;
}

// 归一化
bool mathQuaternionNormalize(const mathQuaternion_t *quat, mathQuaternion_t *quat_out)
{
    if (quat == NULL || quat_out == NULL) {
        return false;
    }

    mathQuaternion_t quat_temp;

    quat_temp = *quat;
    if (mathQuaternionNormalizeInPlace(&quat_temp) == false) {
        return false;
    }

    *quat_out = quat_temp;
    return true;
}

bool mathQuaternionNormSquare(const mathQuaternion_t *quat, float *out)
{
    if (quat == NULL || out == NULL) {
        return false;
    }

    *out = mathQuaternionDot(quat, quat);
    return true;
}

bool mathQuaternionNorm(const mathQuaternion_t *quat, float *out)
{
    if (quat == NULL || out == NULL) {
        return false;
    }

    float quat_norm_square;

    quat_norm_square = mathQuaternionDot(quat, quat);
    if (arm_sqrt_f32(quat_norm_square, out) != ARM_MATH_SUCCESS) {
        return false;
    }

    return true;
}

bool mathQuaternionSetIdentity(mathQuaternion_t *quat)
{
    if (quat == NULL) {
        return false;
    }

    quat->q_[MATH_QUATERNION_W] = 1.0f;
    quat->q_[MATH_QUATERNION_X] = 0.0f;
    quat->q_[MATH_QUATERNION_Y] = 0.0f;
    quat->q_[MATH_QUATERNION_Z] = 0.0f;

    return true;
}

// 共轭四元数
bool mathQuaternionConjugate(const mathQuaternion_t *quat, mathQuaternion_t *quat_out)
{
    if (quat == NULL || quat_out == NULL) {
        return false;
    }

    quat_out->q_[MATH_QUATERNION_W] = quat->q_[MATH_QUATERNION_W];
    quat_out->q_[MATH_QUATERNION_X] = -quat->q_[MATH_QUATERNION_X];
    quat_out->q_[MATH_QUATERNION_Y] = -quat->q_[MATH_QUATERNION_Y];
    quat_out->q_[MATH_QUATERNION_Z] = -quat->q_[MATH_QUATERNION_Z];

    return true;
}

// 四元数乘法，请注意顺序
bool mathQuaternionMultiply(const mathQuaternion_t *quat_1, const mathQuaternion_t *quat_2, mathQuaternion_t *quat_out)
{
    if (quat_1 == NULL || quat_2 == NULL || quat_out == NULL) {
        return false;
    }

    mathQuaternion_t quat_temp;

    quat_temp.q_[MATH_QUATERNION_W] = quat_1->q_[MATH_QUATERNION_W] * quat_2->q_[MATH_QUATERNION_W] -
                                      quat_1->q_[MATH_QUATERNION_X] * quat_2->q_[MATH_QUATERNION_X] -
                                      quat_1->q_[MATH_QUATERNION_Y] * quat_2->q_[MATH_QUATERNION_Y] -
                                      quat_1->q_[MATH_QUATERNION_Z] * quat_2->q_[MATH_QUATERNION_Z];
    quat_temp.q_[MATH_QUATERNION_X] = quat_1->q_[MATH_QUATERNION_W] * quat_2->q_[MATH_QUATERNION_X] +
                                      quat_1->q_[MATH_QUATERNION_X] * quat_2->q_[MATH_QUATERNION_W] +
                                      quat_1->q_[MATH_QUATERNION_Y] * quat_2->q_[MATH_QUATERNION_Z] -
                                      quat_1->q_[MATH_QUATERNION_Z] * quat_2->q_[MATH_QUATERNION_Y];
    quat_temp.q_[MATH_QUATERNION_Y] = quat_1->q_[MATH_QUATERNION_W] * quat_2->q_[MATH_QUATERNION_Y] -
                                      quat_1->q_[MATH_QUATERNION_X] * quat_2->q_[MATH_QUATERNION_Z] +
                                      quat_1->q_[MATH_QUATERNION_Y] * quat_2->q_[MATH_QUATERNION_W] +
                                      quat_1->q_[MATH_QUATERNION_Z] * quat_2->q_[MATH_QUATERNION_X];
    quat_temp.q_[MATH_QUATERNION_Z] = quat_1->q_[MATH_QUATERNION_W] * quat_2->q_[MATH_QUATERNION_Z] +
                                      quat_1->q_[MATH_QUATERNION_X] * quat_2->q_[MATH_QUATERNION_Y] -
                                      quat_1->q_[MATH_QUATERNION_Y] * quat_2->q_[MATH_QUATERNION_X] +
                                      quat_1->q_[MATH_QUATERNION_Z] * quat_2->q_[MATH_QUATERNION_W];

    *quat_out = quat_temp;
    return true;
}

bool mathQuaternionBuildFromSmallAngleError(const float angle_error_rad[3], mathQuaternion_t *quat_out)
{
    if (angle_error_rad == NULL || quat_out == NULL) {
        return false;
    }

    float error_norm_square;
    float error_norm;
    float axis[3];
    float error_norm_half;

    error_norm_square = angle_error_rad[0] * angle_error_rad[0] +
                        angle_error_rad[1] * angle_error_rad[1] +
                        angle_error_rad[2] * angle_error_rad[2];
    if (arm_sqrt_f32(error_norm_square, &error_norm) != ARM_MATH_SUCCESS) {
        return false;
    }

    if (error_norm < MATH_QUATERNION_EPSILON) {
        return mathQuaternionSetIdentity(quat_out);
    }

    axis[0] = angle_error_rad[0] / error_norm;
    axis[1] = angle_error_rad[1] / error_norm;
    axis[2] = angle_error_rad[2] / error_norm;

    error_norm_half = 0.5f * error_norm;
    quat_out->q_[MATH_QUATERNION_W] = arm_cos_f32(error_norm_half);
    quat_out->q_[MATH_QUATERNION_X] = axis[0] * arm_sin_f32(error_norm_half);
    quat_out->q_[MATH_QUATERNION_Y] = axis[1] * arm_sin_f32(error_norm_half);
    quat_out->q_[MATH_QUATERNION_Z] = axis[2] * arm_sin_f32(error_norm_half);

    return mathQuaternionNormalizeInPlace(quat_out);
}

bool mathQuaternionBuildFromSmallAngleErrorLinear(const float angle_error_rad[3], mathQuaternion_t *quat_out)
{
    if (angle_error_rad == NULL || quat_out == NULL) {
        return false;
    }

    quat_out->q_[MATH_QUATERNION_W] = 1.0f;
    quat_out->q_[MATH_QUATERNION_X] = 0.5f * angle_error_rad[0];
    quat_out->q_[MATH_QUATERNION_Y] = 0.5f * angle_error_rad[1];
    quat_out->q_[MATH_QUATERNION_Z] = 0.5f * angle_error_rad[2];

    return mathQuaternionNormalizeInPlace(quat_out);
}

// 通过小角度误差向量更新四元数，这个小角度误差定义在body系下
bool mathQuaternionUpdateBySmallAngleErrorInPlace(const float angle_error_rad[3], mathQuaternion_t *quat)
{
    if (angle_error_rad == NULL || quat == NULL) {
        return false;
    }

    mathQuaternion_t quat_error;
    mathQuaternion_t quat_new;

    if (mathQuaternionBuildFromSmallAngleError(angle_error_rad, &quat_error) == false) {
        return false;
    }

    // 右乘 quat_error
    if (mathQuaternionMultiply(quat, &quat_error, &quat_new) == false) {
        return false;
    }

    if (mathQuaternionNormalizeInPlace(&quat_new) == false) {
        return false;
    }

    *quat = quat_new;
    return true;
}

// 通过小角度误差向量更新四元数，这个小角度误差定义在body系下
bool mathQuaternionUpdateBySmallAngleError(const float angle_error_rad[3], const mathQuaternion_t *quat, mathQuaternion_t *quat_out)
{
    if (angle_error_rad == NULL || quat == NULL || quat_out == NULL) {
        return false;
    }

    mathQuaternion_t quat_temp;

    quat_temp = *quat;
    if (mathQuaternionUpdateBySmallAngleErrorInPlace(angle_error_rad, &quat_temp) == false) {
        return false;
    }

    *quat_out = quat_temp;
    return true;
}

// 从四元数构造旋转矩阵，行优先存储
bool mathQuaternionToRotationMatrix(const mathQuaternion_t *quat, float matrix_out[9])
{
    if (quat == NULL || matrix_out == NULL) {
        return false;
    }

    mathQuaternion_t quat_temp;
    float xx;
    float yy;
    float zz;
    float xy;
    float xz;
    float yz;
    float wx;
    float wy;
    float wz;

    if (mathQuaternionNormalize(quat, &quat_temp) == false) {
        return false;
    }

    xx = quat_temp.q_[MATH_QUATERNION_X] * quat_temp.q_[MATH_QUATERNION_X];
    yy = quat_temp.q_[MATH_QUATERNION_Y] * quat_temp.q_[MATH_QUATERNION_Y];
    zz = quat_temp.q_[MATH_QUATERNION_Z] * quat_temp.q_[MATH_QUATERNION_Z];
    xy = quat_temp.q_[MATH_QUATERNION_X] * quat_temp.q_[MATH_QUATERNION_Y];
    xz = quat_temp.q_[MATH_QUATERNION_X] * quat_temp.q_[MATH_QUATERNION_Z];
    yz = quat_temp.q_[MATH_QUATERNION_Y] * quat_temp.q_[MATH_QUATERNION_Z];
    wx = quat_temp.q_[MATH_QUATERNION_W] * quat_temp.q_[MATH_QUATERNION_X];
    wy = quat_temp.q_[MATH_QUATERNION_W] * quat_temp.q_[MATH_QUATERNION_Y];
    wz = quat_temp.q_[MATH_QUATERNION_W] * quat_temp.q_[MATH_QUATERNION_Z];

    matrix_out[0] = 1.0f - 2.0f * (yy + zz);
    matrix_out[1] = 2.0f * (xy - wz);
    matrix_out[2] = 2.0f * (xz + wy);
    matrix_out[3] = 2.0f * (xy + wz);
    matrix_out[4] = 1.0f - 2.0f * (xx + zz);
    matrix_out[5] = 2.0f * (yz - wx);
    matrix_out[6] = 2.0f * (xz - wy);
    matrix_out[7] = 2.0f * (yz + wx);
    matrix_out[8] = 1.0f - 2.0f * (xx + yy);

    return true;
}

// 四元数旋转向量
bool mathQuaternionRotateVector3(const mathQuaternion_t *quat, const float vec3[3], float vec3_out[3])
{
    if (quat == NULL || vec3 == NULL || vec3_out == NULL) {
        return false;
    }

    mathQuaternion_t quat_temp;
    float matrix[9];

    if (mathQuaternionNormalize(quat, &quat_temp) == false) {
        return false;
    }

    if (mathQuaternionToRotationMatrix(&quat_temp, matrix) == false) {
        return false;
    }

    vec3_out[0] = matrix[0] * vec3[0] + matrix[1] * vec3[1] + matrix[2] * vec3[2];
    vec3_out[1] = matrix[3] * vec3[0] + matrix[4] * vec3[1] + matrix[5] * vec3[2];
    vec3_out[2] = matrix[6] * vec3[0] + matrix[7] * vec3[1] + matrix[8] * vec3[2];

    return true;
}

bool mathQuaternionToEulerZYX(const mathQuaternion_t *quat, float euler_zyx_rad_out[3])
{
    if (quat == NULL || euler_zyx_rad_out == NULL) {
        return false;
    }

    mathQuaternion_t quat_temp;
    float qw;
    float qx;
    float qy;
    float qz;
    float sin_pitch;

    if (mathQuaternionNormalize(quat, &quat_temp) == false) {
        return false;
    }

    qw = quat_temp.q_[MATH_QUATERNION_W];
    qx = quat_temp.q_[MATH_QUATERNION_X];
    qy = quat_temp.q_[MATH_QUATERNION_Y];
    qz = quat_temp.q_[MATH_QUATERNION_Z];

    euler_zyx_rad_out[0] = atan2f(2.0f * (qw * qz + qx * qy),
                                  1.0f - 2.0f * (qy * qy + qz * qz));

    sin_pitch = 2.0f * (qw * qy - qz * qx);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    euler_zyx_rad_out[1] = asinf(sin_pitch);

    euler_zyx_rad_out[2] = atan2f(2.0f * (qw * qx + qy * qz),
                                  1.0f - 2.0f * (qx * qx + qy * qy));

    return true;
}
