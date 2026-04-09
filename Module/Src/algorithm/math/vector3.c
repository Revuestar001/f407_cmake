#include "arm_math.h"

#include <stdbool.h>
#include <string.h>

#include "vector3.h"

#define MATH_VECTOR3_EPSILON 1e-6f

bool mathVec3Dot(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, float *out)
{
    if (vec3_1 == NULL || vec3_2 == NULL || out == NULL) {
        return false;
    }

    *out = vec3_1->v_[MATH_VECTOR3_X] * vec3_2->v_[MATH_VECTOR3_X] +
           vec3_1->v_[MATH_VECTOR3_Y] * vec3_2->v_[MATH_VECTOR3_Y] +
           vec3_1->v_[MATH_VECTOR3_Z] * vec3_2->v_[MATH_VECTOR3_Z];
    return true;
}

bool mathVec3Norm(const mathVector3_t *vec3, float *out)
{
    if (vec3 == NULL || out == NULL) {
        return false;
    }

    float vec3_norm_square;

    if (mathVec3Dot(vec3, vec3, &vec3_norm_square) == false) {
        return false;
    }
    if (arm_sqrt_f32(vec3_norm_square, out) != ARM_MATH_SUCCESS) {
        return false;
    }

    return true;
}

bool mathVec3Normalize(const mathVector3_t *vec3, mathVector3_t *vec3_out)
{
    if (vec3 == NULL || vec3_out == NULL) {
        return false;
    }

    mathVector3_t vec3_temp;
    float vec3_norm;

    if (mathVec3Norm(vec3, &vec3_norm) == false) {
        return false;
    }

    if (vec3_norm < MATH_VECTOR3_EPSILON) {
        return false;
    }

    vec3_temp = *vec3;
    vec3_temp.v_[MATH_VECTOR3_X] /= vec3_norm;
    vec3_temp.v_[MATH_VECTOR3_Y] /= vec3_norm;
    vec3_temp.v_[MATH_VECTOR3_Z] /= vec3_norm;

    *vec3_out = vec3_temp;
    return true;
}

bool mathVec3NormalizeInPlace(mathVector3_t *vec3)
{
    if (vec3 == NULL) {
        return false;
    }

    mathVector3_t vec3_temp;

    if (mathVec3Normalize(vec3, &vec3_temp) == false) {
        return false;
    }
    
    *vec3 = vec3_temp;
    return true;
}

bool mathVec3Add(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, mathVector3_t *vec3_out)
{
    if (vec3_1 == NULL || vec3_2 == NULL || vec3_out == NULL) {
        return false;
    }

    vec3_out->v_[MATH_VECTOR3_X] = vec3_1->v_[MATH_VECTOR3_X] + vec3_2->v_[MATH_VECTOR3_X];
    vec3_out->v_[MATH_VECTOR3_Y] = vec3_1->v_[MATH_VECTOR3_Y] + vec3_2->v_[MATH_VECTOR3_Y];
    vec3_out->v_[MATH_VECTOR3_Z] = vec3_1->v_[MATH_VECTOR3_Z] + vec3_2->v_[MATH_VECTOR3_Z];
    return true;
}

bool mathVec3Subtract(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, mathVector3_t *vec3_out)
{
    if (vec3_1 == NULL || vec3_2 == NULL || vec3_out == NULL) {
        return false;
    }

    vec3_out->v_[MATH_VECTOR3_X] = vec3_1->v_[MATH_VECTOR3_X] - vec3_2->v_[MATH_VECTOR3_X];
    vec3_out->v_[MATH_VECTOR3_Y] = vec3_1->v_[MATH_VECTOR3_Y] - vec3_2->v_[MATH_VECTOR3_Y];
    vec3_out->v_[MATH_VECTOR3_Z] = vec3_1->v_[MATH_VECTOR3_Z] - vec3_2->v_[MATH_VECTOR3_Z];
    return true;
}

bool mathVec3Scale(const mathVector3_t *vec3, float scale, mathVector3_t *vec3_out)
{
    if (vec3 == NULL || vec3_out == NULL) {
        return false;
    }

    vec3_out->v_[MATH_VECTOR3_X] = vec3->v_[MATH_VECTOR3_X] * scale;
    vec3_out->v_[MATH_VECTOR3_Y] = vec3->v_[MATH_VECTOR3_Y] * scale;
    vec3_out->v_[MATH_VECTOR3_Z] = vec3->v_[MATH_VECTOR3_Z] * scale;
    return true;
}

bool mathVec3Cross(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, mathVector3_t *vec3_out)
{
    if (vec3_1 == NULL || vec3_2 == NULL || vec3_out == NULL) {
        return false;
    }

    mathVector3_t vec3_temp;

    vec3_temp.v_[MATH_VECTOR3_X] = vec3_1->v_[MATH_VECTOR3_Y] * vec3_2->v_[MATH_VECTOR3_Z] -
                                   vec3_1->v_[MATH_VECTOR3_Z] * vec3_2->v_[MATH_VECTOR3_Y];
    vec3_temp.v_[MATH_VECTOR3_Y] = vec3_1->v_[MATH_VECTOR3_Z] * vec3_2->v_[MATH_VECTOR3_X] -
                                   vec3_1->v_[MATH_VECTOR3_X] * vec3_2->v_[MATH_VECTOR3_Z];
    vec3_temp.v_[MATH_VECTOR3_Z] = vec3_1->v_[MATH_VECTOR3_X] * vec3_2->v_[MATH_VECTOR3_Y] -
                                   vec3_1->v_[MATH_VECTOR3_Y] * vec3_2->v_[MATH_VECTOR3_X];

    *vec3_out = vec3_temp;
    return true;
}

bool mathVec3BuildSkewSymmetricMatrix(const mathVector3_t *vec3, float matrix_out[9])
{
    if (vec3 == NULL || matrix_out == NULL) {
        return false;
    }

    matrix_out[0] = 0.0f;
    matrix_out[1] = -vec3->v_[MATH_VECTOR3_Z];
    matrix_out[2] = vec3->v_[MATH_VECTOR3_Y];

    matrix_out[3] = vec3->v_[MATH_VECTOR3_Z];
    matrix_out[4] = 0.0f;
    matrix_out[5] = -vec3->v_[MATH_VECTOR3_X];

    matrix_out[6] = -vec3->v_[MATH_VECTOR3_Y];
    matrix_out[7] = vec3->v_[MATH_VECTOR3_X];
    matrix_out[8] = 0.0f;

    return true;
}
