#pragma once

#include <stdbool.h>

typedef struct vector3
{
    float v_[3];
} mathVector3_t;

enum
{
    MATH_VECTOR3_X = 0,
    MATH_VECTOR3_Y,
    MATH_VECTOR3_Z,
};

bool mathVec3Norm(const mathVector3_t *vec3, float *out);
bool mathVec3NormRaw(const float *vec3, float *out);
bool mathVec3Normalize(const mathVector3_t *vec3, mathVector3_t *vec3_out);
bool mathVec3NormalizeInPlace(mathVector3_t *vec3);
bool mathVec3NormalizeInPlaceRaw(float *vec3);
bool mathVec3Dot(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, float *out);
bool mathVec3Add(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, mathVector3_t *vec3_out);
bool mathVec3Subtract(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, mathVector3_t *vec3_out);
bool mathVec3Scale(const mathVector3_t *vec3, float scale, mathVector3_t *vec3_out);
bool mathVec3Cross(const mathVector3_t *vec3_1, const mathVector3_t *vec3_2, mathVector3_t *vec3_out);
bool mathVec3BuildSkewSymmetricMatrix(const mathVector3_t *vec3, float matrix_out[9]);
