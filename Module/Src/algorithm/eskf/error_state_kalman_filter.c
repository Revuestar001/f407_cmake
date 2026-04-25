#include "arm_math.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "error_state_kalman_filter.h"
#include "matrix.h"
#include "quaternion.h"
#include "vector3.h"
#include "angle.h"

static bool computeQDiscrete(algorithmESKF_t *instance, float dt)
{
    if (instance == NULL) {
        return false;
    }

    // 对角线元素
    float val_angle_angle = instance->params_.gyro_noise_rads_sqrt_hz_ * instance->params_.gyro_noise_rads_sqrt_hz_ * dt + 
                            instance->params_.gyro_random_walk_rads2_sqrt_hz_ * instance->params_.gyro_random_walk_rads2_sqrt_hz_ * dt * dt * dt / 3.0f;
    float val_gyro_gyro = instance->params_.gyro_random_walk_rads2_sqrt_hz_ * instance->params_.gyro_random_walk_rads2_sqrt_hz_ * dt;
    // 非对角对称
    float val_angle_gyro = -0.5f * instance->params_.gyro_random_walk_rads2_sqrt_hz_ * instance->params_.gyro_random_walk_rads2_sqrt_hz_ * dt * dt;
    mathMatrixSetZero(&instance->Q_);
    mathMatrixSetBlockDiagScalar(&instance->Q_, 
                                0U, 
                                0U, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                val_angle_angle);
    mathMatrixSetBlockDiagScalar(&instance->Q_, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, 
                                ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, 
                                val_gyro_gyro);
    mathMatrixSetBlockDiagScalar(&instance->Q_, 
                                0U, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, 
                                val_angle_gyro);
    mathMatrixSetBlockDiagScalar(&instance->Q_, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                0U, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, 
                                val_angle_gyro);

    return true;
}

static bool checkAccelMeasurementNormGate(const algorithmESKF_t *instance,
                                          const float measurement[ALGORITHM_ESKF_MEASURE_ACCEL_DIM],
                                          bool *pass_gate)
{
    if (instance == NULL || measurement == NULL || instance->gravity_ref_n_.pData == NULL || pass_gate == NULL) {
        return false;
    }

    mathVector3_t accel_measure_vec3 = {0};
    mathVector3_t gravity_ref_n_vec3 = {0};
    float accel_measure_norm = 0.0f;
    float gravity_ref_norm = 0.0f;
    float accel_norm_error = 0.0f;

    memcpy(accel_measure_vec3.v_, measurement, sizeof(accel_measure_vec3.v_));
    memcpy(gravity_ref_n_vec3.v_, instance->gravity_ref_n_.pData, sizeof(gravity_ref_n_vec3.v_));

    if (mathVec3Norm(&accel_measure_vec3, &accel_measure_norm) == false) {
        return false;
    }

    if (mathVec3Norm(&gravity_ref_n_vec3, &gravity_ref_norm) == false) {
        return false;
    }

    if (gravity_ref_norm <= 0.0f) {
        return false;
    }

    accel_norm_error = fabsf(accel_measure_norm - gravity_ref_norm);

    *pass_gate = (accel_norm_error <= gravity_ref_norm * ALGORITHM_ESKF_ACCEL_NORM_GATE_RATIO);

    return true;
}

static bool checkMagMeasurementNormGate(const algorithmESKF_t *instance,
                                        const float measurement[ALGORITHM_ESKF_MEASURE_MAG_DIM],
                                        bool *pass_gate)
{
    if (instance == NULL || measurement == NULL || pass_gate == NULL) {
        return false;
    }

    mathVector3_t mag_measure_vec3 = {0};
    float mag_measure_norm = 0.0f;
    float mag_norm_error = 0.0f;

    memcpy(mag_measure_vec3.v_, measurement, sizeof(mag_measure_vec3.v_));

    if (mathVec3Norm(&mag_measure_vec3, &mag_measure_norm) == false) {
        return false;
    }

    if (instance->geo_mag_ref_norm_ <= 0.0f) {
        return false;
    }

    mag_norm_error = fabsf(mag_measure_norm - instance->geo_mag_ref_norm_);

    *pass_gate = (mag_norm_error <= instance->geo_mag_ref_norm_ * ALGORITHM_ESKF_MAG_NORM_GATE_RATIO);

    return true;
}

// 卡方检验
static bool computeAccelInnovationChiSquare(const float accel_residual[ALGORITHM_ESKF_MEASURE_ACCEL_DIM],
                                            const mathMatrix_t *S_accel_inv,
                                            float *chi_square_out)
{
    if (accel_residual == NULL || S_accel_inv == NULL || S_accel_inv->pData == NULL || chi_square_out == NULL) {
        return false;
    }

    if (S_accel_inv->numRows != ALGORITHM_ESKF_MEASURE_ACCEL_DIM || S_accel_inv->numCols != ALGORITHM_ESKF_MEASURE_ACCEL_DIM) {
        return false;
    }

    float weighted_residual[ALGORITHM_ESKF_MEASURE_ACCEL_DIM] = {0.0f};
    *chi_square_out = 0.0f;

    for (size_t r = 0; r < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; r++) {
        for (size_t c = 0; c < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; c++) {
            weighted_residual[r] += S_accel_inv->pData[r * S_accel_inv->numCols + c] * accel_residual[c];
        }
    }

    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; i++) {
        *chi_square_out += accel_residual[i] * weighted_residual[i];
    }

    return true;
}

static bool computeMagInnovationChiSquare(const float mag_residual[ALGORITHM_ESKF_MEASURE_MAG_DIM],
                                          const mathMatrix_t *S_mag_inv,
                                          float *chi_square_out)
{
    if (mag_residual == NULL || S_mag_inv == NULL || S_mag_inv->pData == NULL || chi_square_out == NULL) {
        return false;
    }

    if (S_mag_inv->numRows != ALGORITHM_ESKF_MEASURE_MAG_DIM || S_mag_inv->numCols != ALGORITHM_ESKF_MEASURE_MAG_DIM) {
        return false;
    }

    float weighted_residual[ALGORITHM_ESKF_MEASURE_MAG_DIM] = {0.0f};
    *chi_square_out = 0.0f;

    for (size_t r = 0; r < ALGORITHM_ESKF_MEASURE_MAG_DIM; r++) {
        for (size_t c = 0; c < ALGORITHM_ESKF_MEASURE_MAG_DIM; c++) {
            weighted_residual[r] += S_mag_inv->pData[r * S_mag_inv->numCols + c] * mag_residual[c];
        }
    }

    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_MAG_DIM; i++) {
        *chi_square_out += mag_residual[i] * weighted_residual[i];
    }

    return true;
}

// 固定维度矩阵 helper：直接针对 9x9 / 9x3 / 3x9 做实现，减少通用矩阵接口调度开销。
// 原地对称化
static void eskfMat9x9SymmetrizeInPlace(float *mat_9x9)
{
    if (mat_9x9 == NULL) {
        return;
    }

    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        for (size_t c = r + 1U; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
            float sym_val = 0.5f * (mat_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] +
                                    mat_9x9[c * ALGORITHM_ESKF_ERROR_STATE_DIM + r]);
            mat_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] = sym_val;
            mat_9x9[c * ALGORITHM_ESKF_ERROR_STATE_DIM + r] = sym_val;
        }
    }
}

static void eskfMat9x9Mul(const float *lhs_9x9, const float *rhs_9x9, float *out_9x9)
{
    memset(out_9x9, 0, sizeof(float) * ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM);

    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        float *out_row = &out_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];
        const float *lhs_row = &lhs_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];

        for (size_t k = 0; k < ALGORITHM_ESKF_ERROR_STATE_DIM; k++) {
            float lhs_val = lhs_row[k];
            const float *rhs_row = &rhs_9x9[k * ALGORITHM_ESKF_ERROR_STATE_DIM];

            for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
                out_row[c] += lhs_val * rhs_row[c];
            }
        }
    }
}

// A * B^T
static void eskfMat9x9MulTransB(const float *lhs_9x9, const float *rhs_9x9, float *out_9x9)
{
    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        const float *lhs_row = &lhs_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];

        for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
            const float *rhs_row = &rhs_9x9[c * ALGORITHM_ESKF_ERROR_STATE_DIM];
            float sum = 0.0f;

            for (size_t k = 0; k < ALGORITHM_ESKF_ERROR_STATE_DIM; k++) {
                sum += lhs_row[k] * rhs_row[k];
            }

            out_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] = sum;
        }
    }
}

// A * B^T
static void eskfMat9x9Mul9x3TransB(const float *lhs_9x9, const float *rhs_3x9, float *out_9x3)
{
    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        const float *lhs_row = &lhs_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];

        for (size_t c = 0; c < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; c++) {
            const float *rhs_row = &rhs_3x9[c * ALGORITHM_ESKF_ERROR_STATE_DIM];
            float sum = 0.0f;

            for (size_t k = 0; k < ALGORITHM_ESKF_ERROR_STATE_DIM; k++) {
                sum += lhs_row[k] * rhs_row[k];
            }

            out_9x3[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM + c] = sum;
        }
    }
}

static void eskfMat3x9Mul9x9(const float *lhs_3x9, const float *rhs_9x9, float *out_3x9)
{
    memset(out_3x9, 0, sizeof(float) * ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM);

    for (size_t r = 0; r < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; r++) {
        float *out_row = &out_3x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];
        const float *lhs_row = &lhs_3x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];

        for (size_t k = 0; k < ALGORITHM_ESKF_ERROR_STATE_DIM; k++) {
            float lhs_val = lhs_row[k];
            const float *rhs_row = &rhs_9x9[k * ALGORITHM_ESKF_ERROR_STATE_DIM];

            for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
                out_row[c] += lhs_val * rhs_row[c];
            }
        }
    }
}

static void eskfMat3x9Mul9x3(const float *lhs_3x9, const float *rhs_9x3, float *out_3x3)
{
    memset(out_3x3, 0, sizeof(float) * ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM);

    for (size_t r = 0; r < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; r++) {
        float *out_row = &out_3x3[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
        const float *lhs_row = &lhs_3x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];

        for (size_t k = 0; k < ALGORITHM_ESKF_ERROR_STATE_DIM; k++) {
            float lhs_val = lhs_row[k];

            for (size_t c = 0; c < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; c++) {
                out_row[c] += lhs_val * rhs_9x3[k * ALGORITHM_ESKF_MEASURE_ACCEL_DIM + c];
            }
        }
    }
}

static void eskfMat9x3Mul3x3(const float *lhs_9x3, const float *rhs_3x3, float *out_9x3)
{
    memset(out_9x3, 0, sizeof(float) * ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM);

    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        float *out_row = &out_9x3[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
        const float *lhs_row = &lhs_9x3[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];

        for (size_t k = 0; k < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; k++) {
            float lhs_val = lhs_row[k];
            const float *rhs_row = &rhs_3x3[k * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];

            for (size_t c = 0; c < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; c++) {
                out_row[c] += lhs_val * rhs_row[c];
            }
        }
    }
}

static void eskfMat9x3Mul3x1(const float *lhs_9x3, const float *rhs_3x1, float *out_9x1)
{
    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        const float *lhs_row = &lhs_9x3[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
        float sum = 0.0f;

        for (size_t k = 0; k < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; k++) {
            sum += lhs_row[k] * rhs_3x1[k];
        }

        out_9x1[r] = sum;
    }
}

static void eskfMat9x3Mul3x9(const float *lhs_9x3, const float *rhs_3x9, float *out_9x9)
{
    memset(out_9x9, 0, sizeof(float) * ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM);

    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        float *out_row = &out_9x9[r * ALGORITHM_ESKF_ERROR_STATE_DIM];
        const float *lhs_row = &lhs_9x3[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];

        for (size_t k = 0; k < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; k++) {
            float lhs_val = lhs_row[k];
            const float *rhs_row = &rhs_3x9[k * ALGORITHM_ESKF_ERROR_STATE_DIM];

            for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
                out_row[c] += lhs_val * rhs_row[c];
            }
        }
    }
}

// 直接构造PHI = I + F * dt，避免构造中间矩阵
static void eskfBuildPhiFirstOrder(const float *F_9x9, float dt, float *PHI_9x9)
{
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM; i++) {
        PHI_9x9[i] = F_9x9[i] * dt;
    }

    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DIM; i++) {
        PHI_9x9[i * ALGORITHM_ESKF_ERROR_STATE_DIM + i] += 1.0f;
    }
}

static bool updatePosteriorP3DJosephOptimized(algorithmESKF_t *instance,
                                              const float *K_9x3,
                                              const float *H_3x9,
                                              const float *S_3x3)
{
    if (instance == NULL || K_9x3 == NULL || H_3x9 == NULL || S_3x3 == NULL) {
        return false;
    }

    // 优化原理：Joseph 形式按低秩展开，避免显式构造 (I-KH) 并做多次完整 9x9 乘法。
    // 等价形式：P_posterior = P_priori - (P_priori h^T) (P_priori h^T)^T / S
    eskfMat3x9Mul9x9(H_3x9, instance->P_data_, instance->temp_mat_3x9_0_data_);
    eskfMat9x3Mul3x9(K_9x3, instance->temp_mat_3x9_0_data_, instance->temp_mat_9x9_0_data_);
    eskfMat9x3Mul3x3(K_9x3, S_3x3, instance->temp_mat_9x3_0_data_);

    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
            float ksk_t = 0.0f;
            for (size_t k = 0; k < ALGORITHM_ESKF_MEASURE_ACCEL_DIM; k++) {
                ksk_t += instance->temp_mat_9x3_0_data_[r * ALGORITHM_ESKF_MEASURE_ACCEL_DIM + k] *
                         K_9x3[c * ALGORITHM_ESKF_MEASURE_ACCEL_DIM + k];
            }

            instance->P_data_[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] =
                instance->P_data_[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] -
                instance->temp_mat_9x9_0_data_[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] -
                instance->temp_mat_9x9_0_data_[c * ALGORITHM_ESKF_ERROR_STATE_DIM + r] +
                ksk_t;
        }
    }

    // P_posterior是对称矩阵
    eskfMat9x9SymmetrizeInPlace(instance->P_data_);
    return true;
}

static bool updatePosteriorPScalarJosephOptimized(algorithmESKF_t *instance,
                                                  const float *P_H_T_9x1,
                                                  float S_scalar)
{
    if (instance == NULL || P_H_T_9x1 == NULL || S_scalar <= 1.0e-6f) {
        return false;
    }

    // 优化原理：标量观测下 Joseph 形式可化为秩1更新，避免完整 9x9 构造和乘法。
    // 等价形式：P_posterior = P_priori - (P_priori h^T) (P_priori h^T)^T / S
    float inv_S = 1.0f / S_scalar;
    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
            instance->P_data_[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] -=
                P_H_T_9x1[r] * P_H_T_9x1[c] * inv_S;
        }
    }

    // P_posterior是对称矩阵
    eskfMat9x9SymmetrizeInPlace(instance->P_data_);
    return true;
}

static bool injectErrorToNominal(algorithmESKF_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    float angle_error_rad[ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM];
    memcpy(angle_error_rad, instance->error_states_data_, sizeof(angle_error_rad));
    
    // 注入误差，计算后验名义四元数
    if (mathQuaternionUpdateBySmallAngleErrorInPlace(angle_error_rad, &instance->nominal_state_quat_) == false) {
        return false;
    }

    // 注入误差，计算后验名义bias gyro
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM; i ++) {
        instance->nomial_state_bias_gyro_data_[i] += instance->error_states_data_[ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM + i];
    }

    return true;
}

static bool updatePosteriorPAccel(algorithmESKF_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    // 优化原理：accel 是 3 维观测，这里直接走 9x3/3x9 的低秩 Joseph 展开。
    return updatePosteriorP3DJosephOptimized(instance,
                                             instance->K_accel_data_,
                                             instance->H_accel_data_,
                                             instance->S_accel_data_);
}

static bool updatePosteriorPMag(algorithmESKF_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    // 优化原理：full mag 同样是 3 维观测，直接复用低秩 Joseph 展开。
    return updatePosteriorP3DJosephOptimized(instance,
                                             instance->K_mag_data_,
                                             instance->H_mag_data_,
                                             instance->S_mag_data_);
}

static bool updatePosteriorPMagYawOnly(algorithmESKF_t *instance,
                                       const float *P_H_mag_yaw_T_9x1,
                                       float S_mag_yaw)
{
    if (instance == NULL || instance->is_initialized_ == false || P_H_mag_yaw_T_9x1 == NULL) {
        return false;
    }

    return updatePosteriorPScalarJosephOptimized(instance, P_H_mag_yaw_T_9x1, S_mag_yaw);
}

static bool clearErrorStates(algorithmESKF_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    memset(instance->error_states_data_, 0, sizeof(instance->error_states_data_));

    return true;
}

bool algorithmESKFInit(algorithmESKF_t *instance, algorithmESKFParams_t *params)
{
    if (instance == NULL || params == NULL) {
        return false;
    }

    memset(instance, 0, sizeof(algorithmESKF_t));

    // 初始化所有矩阵
    mathMatrixInit(&instance->nomial_state_bias_gyro_, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, 1U, instance->nomial_state_bias_gyro_data_);
    mathMatrixInit(&instance->error_states_, ALGORITHM_ESKF_ERROR_STATE_DIM, 1U, instance->error_states_data_);

    mathMatrixInit(&instance->P_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->P_data_);
    mathMatrixInit(&instance->Q_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->Q_data_);
    mathMatrixInit(&instance->R_accel_, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, instance->R_accel_data_);
    mathMatrixInit(&instance->R_mag_, ALGORITHM_ESKF_MEASURE_MAG_DIM, ALGORITHM_ESKF_MEASURE_MAG_DIM, instance->R_mag_data_);

    mathMatrixInit(&instance->F_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->F_data_);
    mathMatrixInit(&instance->PHI_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->PHI_data_);
    mathMatrixInit(&instance->G_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->G_data_);
    mathMatrixInit(&instance->H_accel_, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->H_accel_data_);
    mathMatrixInit(&instance->H_mag_, ALGORITHM_ESKF_MEASURE_MAG_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, instance->H_mag_data_);
    mathMatrixInit(&instance->S_accel_, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, instance->S_accel_data_);
    mathMatrixInit(&instance->S_mag_, ALGORITHM_ESKF_MEASURE_MAG_DIM, ALGORITHM_ESKF_MEASURE_MAG_DIM, instance->S_mag_data_);
    mathMatrixInit(&instance->K_accel_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, instance->K_accel_data_);
    mathMatrixInit(&instance->K_mag_, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_MEASURE_MAG_DIM, instance->K_mag_data_);

    instance->params_ = *params;

    // 设置初始状态
    instance->nominal_state_quat_ = params->init_params_.quat_init_;
    if (mathQuaternionNormalizeInPlace(&instance->nominal_state_quat_) == false) {
        return false;
    }
    memcpy(instance->nomial_state_bias_gyro_data_, params->init_params_.gyro_bias_init_, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM * sizeof(float));

    // 设置初始状态误差协方差矩阵
    float p_diag_init[ALGORITHM_ESKF_ERROR_STATE_DIM];
    memcpy(p_diag_init, params->init_params_.angle_error_variance_, ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM * sizeof(float));
    memcpy(p_diag_init + ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, params->init_params_.delta_bias_gyro_variance_, ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM * sizeof(float));
    if (mathMatrixSetDiag(&instance->P_, p_diag_init, ALGORITHM_ESKF_ERROR_STATE_DIM) == false) {
        return false;
    }

    // 设置测量噪声协方差矩阵R,这里是工程近似,元素值val = (连续时间噪声密度^2 * 采样频率f * 0.5) * 放大系数
    float r_accel_init_val = params->accel_noise_ms2_sqrt_hz_ * params->accel_noise_ms2_sqrt_hz_ * ALGORITHM_ESKF_IMU_SAMPLE_FREQUENCY * 0.5f * ALGORITHM_ESKF_R_ACCEL_SCALE_FACTOR;
    float r_mag_noise_density = params->mag_noise_ut_sqrt_hz_;
    mathVector3_t geo_mag_ref_vec_n = {0};
    memcpy(geo_mag_ref_vec_n.v_, instance->params_.init_params_.geo_mag_ref_dir_n_, sizeof(geo_mag_ref_vec_n.v_));
    if (mathVec3Norm(&geo_mag_ref_vec_n, &instance->geo_mag_ref_norm_) == false || instance->geo_mag_ref_norm_ <= 0.0f) {
        return false;
    }
    if (instance->params_.mag_measurement_mode_ == ALGORITHM_ESKF_MAG_MEASUREMENT_NORMALIZED_VECTOR) {
        if (mathVec3NormalizeInPlace(&geo_mag_ref_vec_n) == false) {
            return false;
        }
        memcpy(instance->params_.init_params_.geo_mag_ref_dir_n_, geo_mag_ref_vec_n.v_, sizeof(geo_mag_ref_vec_n.v_));
        r_mag_noise_density /= instance->geo_mag_ref_norm_;
    }
    float r_mag_init_val = r_mag_noise_density * r_mag_noise_density * ALGORITHM_ESKF_MAG_SAMPLE_FREQUENCY * 0.5f * ALGORITHM_ESKF_R_MAG_SCALE_FACTOR;
    if (mathMatrixSetDiagScalar(&instance->R_accel_, r_accel_init_val) == false) {
        return false;
    }
    if (mathMatrixSetDiagScalar(&instance->R_mag_, r_mag_init_val) == false) {
        return false;
    }

    // 设置误差状态转移矩阵PHI为单位矩阵,为啥？
    if (mathMatrixSetIdentity(&instance->PHI_) == false) {
        return false;
    }

    // 设置过程噪声协方差矩阵Q为0
    if (mathMatrixSetZero(&instance->Q_) == false) {
        return false;
    }

    // 存储重力(比力)参考和地磁参考向量
    mathMatrixInit(&instance->gravity_ref_n_, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, 1U, instance->params_.init_params_.gravity_ref_n_);
    mathMatrixInit(&instance->geo_mag_ref_dir_n_, ALGORITHM_ESKF_MEASURE_MAG_DIM, 1U, instance->params_.init_params_.geo_mag_ref_dir_n_);
    
    // 初始化时,dt给0
    instance->dt_ = 0.0f;

    instance->is_initialized_ = true;

    return true;
}

// measurement必须是不包含在线bias校正的，否则eskf这里会重复去偏bias
bool algorithmESKFGyroPredict(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM], float dt)
{
    if (instance == NULL || measurement == NULL || instance->is_initialized_ == false) {
        return false;
    }

    // 防止dt数值异常
    dt = dt < ALGORITHM_ESKF_MIN_DT_S ? ALGORITHM_ESKF_MIN_DT_S : dt;
    dt = dt > ALGORITHM_ESKF_MAX_DT_S ? ALGORITHM_ESKF_MAX_DT_S : dt;

    //
    // 名义状态传播
    //
    // 使用名义状态，角速度测量值去偏
    float omega_hat_priori[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM];
    for (size_t i = 0; i < ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM; i ++) {
        omega_hat_priori[i] = measurement[i] - instance->nomial_state_bias_gyro_data_[i];
    }

    // 计算角增量
    float delta_theta[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM];
    for (size_t i = 0; i < ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM; i ++) {
        delta_theta[i] = omega_hat_priori[i] * dt;
    }

    // 计算名义状态传播后的先验标称姿态四元数
    mathQuaternion_t quat_priori;
    // 这里可能数值不稳，也许使用线性近似比较好
    if (mathQuaternionUpdateBySmallAngleError(delta_theta, &instance->nominal_state_quat_, &quat_priori) == false) {
        return false;
    }
    instance->nominal_state_quat_ = quat_priori;

    // 计算名义状态传播后的 bias,这里采用常值模型，所以保持不变

    //
    // 误差状态连续模型 
    //
    // 计算连续误差状态一阶雅可比矩阵F
    mathVector3_t omega_hat_priori_vec3;
    memcpy(omega_hat_priori_vec3.v_, omega_hat_priori, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM * sizeof(float));
    float omega_hat_priori_skew_[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM * ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM];
    if (mathVec3BuildSkewSymmetricMatrix(&omega_hat_priori_vec3, omega_hat_priori_skew_) == false) {
        return false;
    }
    for (size_t i = 0; i < ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM * ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM; i ++) {
        omega_hat_priori_skew_[i] *= -1.0f;
    }
    mathMatrixSetZero(&instance->F_);
    mathMatrixSetBlock(&instance->F_, 0U, 0U, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, omega_hat_priori_skew_);
    mathMatrixSetBlockDiagScalar(&instance->F_, 0U, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, -1.0f);
    
    // 计算噪声输入矩阵G
    mathMatrixSetIdentity(&instance->G_);
    mathMatrixSetBlockDiagScalar(&instance->G_, 0U, 0U, ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, -1.0f);

    //
    // 离散过程噪声协方差矩阵Q
    //
    if (computeQDiscrete(instance, dt) == false) {
        return false;
    }

    //
    // 计算误差状态转移矩阵PHI
    //
    // 优化原理：固定 9x9 直接构造 PHI = I + F * dt，避免 scale + identity + add 三次通用矩阵调用。
    eskfBuildPhiFirstOrder(instance->F_data_, dt, instance->PHI_data_);

    //
    // 状态误差协方差矩阵P传播
    //
    // 优化原理：固定 9x9 乘法 + A * B^T，省掉额外转置矩阵和通用 arm_mat 路径。
    // 当前先验P = PHI * 当前后验P * PHI^T + Q
    eskfMat9x9Mul(instance->PHI_data_, instance->P_data_, instance->temp_mat_9x9_2_data_);
    eskfMat9x9MulTransB(instance->temp_mat_9x9_2_data_, instance->PHI_data_, instance->temp_mat_9x9_3_data_);
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM; i++) {
        instance->P_data_[i] = instance->temp_mat_9x9_3_data_[i] + instance->Q_data_[i];
    }
    eskfMat9x9SymmetrizeInPlace(instance->P_data_);

    instance->dt_ = dt;

    return true;
}

// 使用accel测量值(只包含offline bias + scale)更新
bool algorithmESKFAccelUpdate(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_MEASURE_ACCEL_DIM], float dt)
{
    if (instance == NULL || measurement == NULL || instance->is_initialized_ == false) {
        return false;
    }

    (void)dt;

    bool accel_norm_gate_pass = false;
    if (checkAccelMeasurementNormGate(instance, measurement, &accel_norm_gate_pass) == false) {
        return false;
    }
    // accel模长门限不通过，本轮不使用accel更新，但不视为滤波失败
    if (accel_norm_gate_pass == false) {
        return true;
    }

    //
    // 预测加速度计测量值
    // 
    // 使用 instance 持有的 temp buffer，减少热路径栈占用。
    // 计算先验姿态旋转矩阵
    mathMatrix_t rotate_matrix_b_to_n;
    mathMatrixInit(&rotate_matrix_b_to_n, 3U, 3U, instance->temp_mat_3x3_0_data_);
    mathQuaternionToRotationMatrix(&instance->nominal_state_quat_, instance->temp_mat_3x3_0_data_);
    mathMatrix_t rotate_matrix_n_to_b;
    mathMatrixInit(&rotate_matrix_n_to_b, 3U, 3U, instance->temp_mat_3x3_1_data_);
    mathMatrixTranspose(&rotate_matrix_b_to_n, &rotate_matrix_n_to_b);

    // 计算预测加速度计测量值
    // accel_measure_predict = C_n_to_b * gravity_vec_n
    mathMatrix_t gravity_vec_b;
    mathMatrixInit(&gravity_vec_b, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, 1U, instance->temp_mat_3x1_0_data_);
    mathMatrixMult(&rotate_matrix_n_to_b, &instance->gravity_ref_n_, &gravity_vec_b);

    // 
    // 计算加速度计测量残差
    //
    // accel_residual = accel_measure - accel_measure_predict
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U; i ++) {
        instance->temp_mat_3x1_2_data_[i] = measurement[i] - instance->temp_mat_3x1_0_data_[i];
    }

    //
    // 计算accel观测/测量一阶雅可比矩阵H
    //
    // 请注意，这里计算H时,gravity_vec_b_skew的负号去掉了才正常，与推导不一致,基本上认为是推导错误！
    if (mathMatrixSetZero(&instance->H_accel_) == false) {
        return false;
    }
    mathVector3_t gravity_vec_b_vec3;
    memcpy(gravity_vec_b_vec3.v_, instance->temp_mat_3x1_0_data_, sizeof(gravity_vec_b_vec3.v_));
    mathMatrix_t gravity_vec_b_skew;
    mathMatrixInit(&gravity_vec_b_skew, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, instance->temp_mat_3x3_2_data_);
    mathVec3BuildSkewSymmetricMatrix(&gravity_vec_b_vec3, instance->temp_mat_3x3_2_data_);
    mathMatrixSetBlockByMatrix(&instance->H_accel_, 0U, 0U, &gravity_vec_b_skew);

    //
    // 计算创新协方差矩阵S
    //
    // S_accel = H_accel * P_priori * H_accel^T + R_accel
    // 优化原理：9x9 * 9x3 和 3x9 * 9x3 都按固定维度展开，减少小矩阵通用接口开销。
    eskfMat9x9Mul9x3TransB(instance->P_data_, instance->H_accel_data_, instance->temp_mat_9x3_1_data_);
    eskfMat3x9Mul9x3(instance->H_accel_data_, instance->temp_mat_9x3_1_data_, instance->temp_mat_3x3_3_data_);
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM; i++) {
        instance->S_accel_data_[i] = instance->temp_mat_3x3_3_data_[i] + instance->R_accel_data_[i];
    }

    //
    // 计算卡尔曼增益K
    //
    // K_accel = P_priori * H_accel^T * S_accel^-1
    // CMSIS-DSP 的 arm_mat_inverse_f32 会修改输入矩阵，因此这里必须先拷贝一份原始 S_accel，
    // 避免把创新协方差本体改成单位阵，导致调试观测和 Joseph 协方差更新都拿到错误的 S。
    mathMatrix_t S_accel_for_inverse;
    mathMatrixInit(&S_accel_for_inverse, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, instance->temp_mat_3x3_5_data_);
    memcpy(instance->temp_mat_3x3_5_data_, instance->S_accel_data_, sizeof(instance->S_accel_data_));

    mathMatrix_t S_accel_inv;
    mathMatrixInit(&S_accel_inv, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, instance->temp_mat_3x3_4_data_);
    if (mathMatrixInverse(&S_accel_for_inverse, &S_accel_inv) != ARM_MATH_SUCCESS) {
        return false;
    }
    // 创新卡方门限不通过，本轮不使用accel更新，但不视为滤波失败
    float accel_innovation_chi_square = 0.0f;
    if (computeAccelInnovationChiSquare(instance->temp_mat_3x1_2_data_, &S_accel_inv, &accel_innovation_chi_square) == false) {
        return false;
    }
    if (accel_innovation_chi_square > ALGORITHM_ESKF_ACCEL_CHI_SQUARE_THRESHOLD) {
        return true;
    }
    eskfMat9x3Mul3x3(instance->temp_mat_9x3_1_data_, instance->temp_mat_3x3_4_data_, instance->K_accel_data_);

    //
    // 计算误差状态后验估计error_states,9x1
    //
    // error_states_posterior = K_accel * accel_residual
    eskfMat9x3Mul3x1(instance->K_accel_data_, instance->temp_mat_3x1_2_data_, instance->error_states_data_);

    // 
    // 注入误差到名义中
    //
    if (injectErrorToNominal(instance) == false) {
        return false;
    }

    //
    // 更新后验误差状态协方差矩阵P
    //
    // joseph形式，P_posterior = (I - K * H) * P_priori * (I - K * H)^T + K * R * K^T
    if (updatePosteriorPAccel(instance) == false) {
        return false;
    }

    //
    // 误差状态清零，后验误差状态协方差矩阵P修正
    //
    if (clearErrorStates(instance) == false) {
        return false;
    }
    // 后验误差状态协方差矩阵P修正暂时不做

    return true;
}

// 使用mag测量值(只包含hard-iron/soft-iron)更新
bool algorithmESKFMagUpdate(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_MEASURE_MAG_DIM], float dt)
{
    if (instance == NULL || measurement == NULL || instance->is_initialized_ == false) {
        return false;
    }

    (void)dt;
    
    //
    // 根据配置选择使用原始磁场向量或归一化方向向量
    //
    mathVector3_t mag_measurement_used;
    memcpy(mag_measurement_used.v_, measurement, sizeof(mag_measurement_used.v_));
    if (instance->params_.mag_measurement_mode_ == ALGORITHM_ESKF_MAG_MEASUREMENT_NORMALIZED_VECTOR) {
        if (mathVec3NormalizeInPlace(&mag_measurement_used) == false) {
            return false;
        }
    }
    
    if (instance->params_.mag_measurement_mode_ == ALGORITHM_ESKF_MAG_MEASUREMENT_RAW_VECTOR) {
        bool mag_norm_gate_pass = false;
        if (checkMagMeasurementNormGate(instance, mag_measurement_used.v_, &mag_norm_gate_pass) == false) {
            return false;
        }
        // mag模长门限不通过，本轮不使用mag更新，但不视为滤波失败
        if (mag_norm_gate_pass == false) {
            return true;
        }
    }
    
    //
    // 预测磁力计测量值
    // 
    // 使用 instance 持有的 temp buffer，减少热路径栈占用。
    // 计算先验姿态旋转矩阵
    mathMatrix_t rotate_matrix_b_to_n;
    mathMatrixInit(&rotate_matrix_b_to_n, 3U, 3U, instance->temp_mat_3x3_0_data_);
    if (mathQuaternionToRotationMatrix(&instance->nominal_state_quat_, instance->temp_mat_3x3_0_data_) == false) {
        return false;
    }
    mathMatrix_t rotate_matrix_n_to_b;
    mathMatrixInit(&rotate_matrix_n_to_b, 3U, 3U, instance->temp_mat_3x3_1_data_);
    mathMatrixTranspose(&rotate_matrix_b_to_n, &rotate_matrix_n_to_b);

    // 计算预测磁力计测量值
    // mag_measure_predict = C_n_to_b * geo_mag_ref_dir
    mathMatrix_t geo_mag_ref_dir_vec_b;
    mathMatrixInit(&geo_mag_ref_dir_vec_b, ALGORITHM_ESKF_MEASURE_MAG_DIM, 1U, instance->temp_mat_3x1_0_data_);
    mathMatrixMult(&rotate_matrix_n_to_b, &instance->geo_mag_ref_dir_n_, &geo_mag_ref_dir_vec_b);
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_MAG_DIM * 1U; i ++) {
        instance->temp_mat_3x1_1_data_[i] = instance->temp_mat_3x1_0_data_[i];
    }

    // 
    // 计算磁力计测量残差
    //
    // mag_residual = mag_measure - mag_measure_predict
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_MAG_DIM * 1U; i ++) {
        instance->temp_mat_3x1_2_data_[i] = mag_measurement_used.v_[i] - instance->temp_mat_3x1_1_data_[i];
    }

    //
    // 计算mag观测/测量一阶雅可比矩阵H
    //
    if (mathMatrixSetZero(&instance->H_mag_) == false) {
        return false;
    }
    mathVector3_t geo_mag_ref_dir_vec_b_vec3;
    memcpy(geo_mag_ref_dir_vec_b_vec3.v_, instance->temp_mat_3x1_0_data_, sizeof(geo_mag_ref_dir_vec_b_vec3.v_));
    mathMatrix_t geo_mag_ref_dir_vec_b_skew;
    mathMatrixInit(&geo_mag_ref_dir_vec_b_skew, ALGORITHM_ESKF_MEASURE_MAG_DIM, ALGORITHM_ESKF_MEASURE_MAG_DIM, instance->temp_mat_3x3_2_data_);
    if (mathVec3BuildSkewSymmetricMatrix(&geo_mag_ref_dir_vec_b_vec3, instance->temp_mat_3x3_2_data_) == false) {
        return false;
    }
    mathMatrixSetBlockByMatrix(&instance->H_mag_, 0U, 0U, &geo_mag_ref_dir_vec_b_skew);

    //
    // 计算创新协方差矩阵S
    //
    // S_mag = H_mag * P_priori * H_mag^T + R_mag
    // 优化原理：9x9 * 9x3 和 3x9 * 9x3 都按固定维度展开，减少小矩阵通用接口开销。
    eskfMat9x9Mul9x3TransB(instance->P_data_, instance->H_mag_data_, instance->temp_mat_9x3_0_data_);
    eskfMat3x9Mul9x3(instance->H_mag_data_, instance->temp_mat_9x3_0_data_, instance->temp_mat_3x3_3_data_);
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_MAG_DIM * ALGORITHM_ESKF_MEASURE_MAG_DIM; i++) {
        instance->S_mag_data_[i] = instance->temp_mat_3x3_3_data_[i] + instance->R_mag_data_[i];
    }

    //
    // 计算卡尔曼增益K
    //
    // K_mag = P_priori * H_mag^T * S_mag^-1
    // CMSIS-DSP 的 arm_mat_inverse_f32 会修改输入矩阵，因此这里必须先拷贝一份原始 S_mag，
    // 避免把创新协方差本体改成单位阵，导致调试观测和 Joseph 协方差更新都拿到错误的 S。
    mathMatrix_t S_mag_for_inverse;
    mathMatrixInit(&S_mag_for_inverse, ALGORITHM_ESKF_MEASURE_MAG_DIM, ALGORITHM_ESKF_MEASURE_MAG_DIM, instance->temp_mat_3x3_5_data_);
    memcpy(instance->temp_mat_3x3_5_data_, instance->S_mag_data_, sizeof(instance->S_mag_data_));

    mathMatrix_t S_mag_inv;
    mathMatrixInit(&S_mag_inv, ALGORITHM_ESKF_MEASURE_MAG_DIM, ALGORITHM_ESKF_MEASURE_MAG_DIM, instance->temp_mat_3x3_4_data_);
    if (mathMatrixInverse(&S_mag_for_inverse, &S_mag_inv) != ARM_MATH_SUCCESS) {
        return false;
    }
    // 创新卡方门限不通过，本轮不使用mag更新，但不视为滤波失败
    float mag_innovation_chi_square = 0.0f;
    if (computeMagInnovationChiSquare(instance->temp_mat_3x1_2_data_, &S_mag_inv, &mag_innovation_chi_square) == false) {
        return false;
    }
    if (mag_innovation_chi_square > ALGORITHM_ESKF_MAG_CHI_SQUARE_THRESHOLD) {
        return true;
    }
    eskfMat9x3Mul3x3(instance->temp_mat_9x3_0_data_, instance->temp_mat_3x3_4_data_, instance->K_mag_data_);

    //
    // 计算误差状态后验估计error_states,9x1
    //
    // error_states_posterior = K_mag * mag_residual
    eskfMat9x3Mul3x1(instance->K_mag_data_, instance->temp_mat_3x1_2_data_, instance->error_states_data_);

    // 
    // 注入误差到名义中
    //
    if (injectErrorToNominal(instance) == false) {
        return false;
    }

    //
    // 更新后验误差状态协方差矩阵P
    //
    // joseph形式，P_posterior = (I - K * H) * P_priori * (I - K * H)^T + K * R * K^T
    if (updatePosteriorPMag(instance) == false) {
        return false;
    }

    //
    // 误差状态清零，后验误差状态协方差矩阵P修正
    //
    if (clearErrorStates(instance) == false) {
        return false;
    }
    // 后验误差状态协方差矩阵P修正暂时不做

    return true;
}

// 使用mag测量值(只包含hard-iron/soft-iron)更新,只用于yaw更新
bool algorithmESKFMagUpdateYawOnly(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_MEASURE_MAG_DIM], float dt)
{
    if (instance == NULL || measurement == NULL || instance->is_initialized_ == false) {
        return false;
    }

    (void)dt;

    //
    // 根据配置选择使用原始磁场向量或归一化方向向量
    //
    mathVector3_t mag_measurement_used;
    memcpy(mag_measurement_used.v_, measurement, sizeof(mag_measurement_used.v_));
    if (instance->params_.mag_measurement_mode_ == ALGORITHM_ESKF_MAG_MEASUREMENT_NORMALIZED_VECTOR) {
        if (mathVec3NormalizeInPlace(&mag_measurement_used) == false) {
            return false;
        }
    }
    mathMatrix_t mag_measure_b;
    // 使用 instance 持有的 temp buffer，减少热路径栈占用。
    mathMatrixInit(&mag_measure_b, ALGORITHM_ESKF_MEASURE_MAG_DIM, 1U, instance->temp_mat_3x1_0_data_);
    memcpy(instance->temp_mat_3x1_0_data_, mag_measurement_used.v_, sizeof(mag_measurement_used.v_));

    //
    // 预测磁力计测量值
    //
    mathMatrix_t rotate_matrix_b_to_n;
    mathMatrixInit(&rotate_matrix_b_to_n, 3U, 3U, instance->temp_mat_3x3_0_data_);
    if (mathQuaternionToRotationMatrix(&instance->nominal_state_quat_, instance->temp_mat_3x3_0_data_) == false) {
        return false;
    }
    mathMatrix_t rotate_matrix_n_to_b;
    mathMatrixInit(&rotate_matrix_n_to_b, 3U, 3U, instance->temp_mat_3x3_1_data_);
    if (mathMatrixTranspose(&rotate_matrix_b_to_n, &rotate_matrix_n_to_b) == false) {
        return false;
    }

    // 当前先验姿态下机体坐标系中的U轴方向
    mathMatrix_t U_axis_n;
    float U_axis_n_data[3U * 1U] = {0.0f, 0.0f, 1.0f};
    mathMatrixInit(&U_axis_n, 3U, 1U, U_axis_n_data);
    mathMatrix_t U_axis_b;
    mathMatrixInit(&U_axis_b, 3U, 1U, instance->temp_mat_3x1_1_data_);
    mathMatrixMult(&rotate_matrix_n_to_b, &U_axis_n, &U_axis_b);
    if (mathVec3NormalizeInPlaceRaw(instance->temp_mat_3x1_1_data_) == false) {
        return false;
    }

    // FL面投影矩阵 P_fl = I - u_b * u_b^T
    mathMatrix_t P_FL_b;
    mathMatrixInit(&P_FL_b, 3U, 3U, instance->temp_mat_3x3_2_data_);
    mathMatrix_t identity;
    mathMatrixInit(&identity, 3U, 3U, instance->temp_mat_3x3_3_data_);
    if (mathMatrixSetIdentity(&identity) == false) {
        return false;
    }
    mathMatrix_t U_axis_T_b;
    mathMatrixInit(&U_axis_T_b, 1U, 3U, instance->temp_mat_1x3_0_data_);
    if (mathMatrixTranspose(&U_axis_b, &U_axis_T_b) == false) {
        return false;
    }
    mathMatrix_t U_axis_U_axis_T_b;
    mathMatrixInit(&U_axis_U_axis_T_b, 3U, 3U, instance->temp_mat_3x3_4_data_);
    mathMatrixMult(&U_axis_b, &U_axis_T_b, &U_axis_U_axis_T_b);
    mathMatrixSub(&identity, &U_axis_U_axis_T_b, &P_FL_b);

    // 参考磁向量投影到FL面
    mathMatrix_t geo_mag_ref_dir_vec_b;
    mathMatrixInit(&geo_mag_ref_dir_vec_b, ALGORITHM_ESKF_MEASURE_MAG_DIM, 1U, instance->temp_mat_3x1_2_data_);
    mathMatrixMult(&rotate_matrix_n_to_b, &instance->geo_mag_ref_dir_n_, &geo_mag_ref_dir_vec_b);
    mathMatrix_t geo_mag_ref_dir_FL_vec_b;
    mathMatrixInit(&geo_mag_ref_dir_FL_vec_b, ALGORITHM_ESKF_MEASURE_MAG_DIM, 1U, instance->temp_mat_3x1_3_data_);
    mathMatrixMult(&P_FL_b, &geo_mag_ref_dir_vec_b, &geo_mag_ref_dir_FL_vec_b);

    float geo_mag_ref_dir_vec_b_norm = 0.0f;
    float geo_mag_ref_dir_FL_vec_b_norm = 0.0f;
    if (mathVec3NormRaw(instance->temp_mat_3x1_2_data_, &geo_mag_ref_dir_vec_b_norm) == false ||
        mathVec3NormRaw(instance->temp_mat_3x1_3_data_, &geo_mag_ref_dir_FL_vec_b_norm) == false) {
        return false;
    }
    if (geo_mag_ref_dir_FL_vec_b_norm < ALGORITHM_ESKF_MAG_YAW_ONLY_REF_NORM_GATE_RATIO * geo_mag_ref_dir_vec_b_norm) {
        return true;
    }
    if (mathVec3NormalizeInPlaceRaw(instance->temp_mat_3x1_3_data_) == false) {
        return false;
    }

    // 测量磁向量投影到FL面
    mathMatrix_t mag_measure_FL_b;
    mathMatrixInit(&mag_measure_FL_b, ALGORITHM_ESKF_MEASURE_MAG_DIM, 1U, instance->temp_mat_3x1_4_data_);
    mathMatrixMult(&P_FL_b, &mag_measure_b, &mag_measure_FL_b);

    float mag_measure_b_norm = 0.0f;
    float mag_measure_FL_b_norm = 0.0f;
    if (mathVec3NormRaw(instance->temp_mat_3x1_0_data_, &mag_measure_b_norm) == false ||
        mathVec3NormRaw(instance->temp_mat_3x1_4_data_, &mag_measure_FL_b_norm) == false) {
        return false;
    }
    if (mag_measure_FL_b_norm < ALGORITHM_ESKF_MAG_YAW_ONLY_MEASURE_NORM_GATE_RATIO * mag_measure_b_norm) {
        return true;
    }
    if (mathVec3NormalizeInPlaceRaw(instance->temp_mat_3x1_4_data_) == false) {
        return false;
    }

    // s = u_b x h_h^b，表示从预测水平磁向量逆时针转正的切向单位向量
    // 请注意，这里叉乘方向需要检查
    mathVector3_t geo_mag_ref_dir_FL_vec_b_vec3 = {0};
    mathVector3_t mag_measure_FL_b_vec3 = {0};
    mathVector3_t U_axis_b_vec3 = {0};
    mathVector3_t s_FL_b_vec3 = {0};
    memcpy(geo_mag_ref_dir_FL_vec_b_vec3.v_, instance->temp_mat_3x1_3_data_, sizeof(geo_mag_ref_dir_FL_vec_b_vec3.v_));
    memcpy(mag_measure_FL_b_vec3.v_, instance->temp_mat_3x1_4_data_, sizeof(mag_measure_FL_b_vec3.v_));
    memcpy(U_axis_b_vec3.v_, instance->temp_mat_3x1_1_data_, sizeof(U_axis_b_vec3.v_));
    if (mathVec3Cross(&U_axis_b_vec3, &geo_mag_ref_dir_FL_vec_b_vec3, &s_FL_b_vec3) == false) {
        return false;
    }
    if (mathVec3NormalizeInPlace(&s_FL_b_vec3) == false) {
        return false;
    }

    //
    // 精确非线性标量残差
    //
    float s_dot_m = 0.0f;
    float h_dot_m = 0.0f;
    if (mathVec3Dot(&s_FL_b_vec3, &mag_measure_FL_b_vec3, &s_dot_m) == false ||
        mathVec3Dot(&geo_mag_ref_dir_FL_vec_b_vec3, &mag_measure_FL_b_vec3, &h_dot_m) == false) {
        return false;
    }
    float psi_residual = mathAngleWrapPi(atan2f(s_dot_m, h_dot_m));

    //
    // 一阶雅可比 H_psi = s_h^T * [h_h^b]_x
    //
    if (mathVec3BuildSkewSymmetricMatrix(&geo_mag_ref_dir_FL_vec_b_vec3, instance->temp_mat_3x3_5_data_) == false) {
        return false;
    }
    memset(instance->temp_mat_1x9_0_data_, 0, sizeof(instance->temp_mat_1x9_0_data_));
    for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM; c++) {
        instance->temp_mat_1x9_0_data_[c] = s_FL_b_vec3.v_[0] * instance->temp_mat_3x3_5_data_[c] +
                                             s_FL_b_vec3.v_[1] * instance->temp_mat_3x3_5_data_[3U + c] +
                                             s_FL_b_vec3.v_[2] * instance->temp_mat_3x3_5_data_[6U + c];
    }

    //
    // 标量创新协方差 S = H P H^T + R
    //
    // 优化原理：标量观测只需要 ph = P * h^T，一个 9x1 即可，不必走通用矩阵乘法。
    for (size_t r = 0; r < ALGORITHM_ESKF_ERROR_STATE_DIM; r++) {
        float sum = 0.0f;
        for (size_t c = 0; c < ALGORITHM_ESKF_ERROR_STATE_DIM; c++) {
            sum += instance->P_data_[r * ALGORITHM_ESKF_ERROR_STATE_DIM + c] *
                   instance->temp_mat_1x9_0_data_[c];
        }
        instance->temp_mat_9x1_1_data_[r] = sum;
    }

    // 测量噪声协方差矩阵R
    // 请注意，这里的写法只适用于磁噪声各向同性(mag_noise_x = mag_noise_y = mag_noise_z)的情况，更严谨的是R_mag_yaw = (s_h * R_mag * s_n^T) / rou^2
    float R_mag_yaw = instance->R_mag_data_[0];
    // 请注意，这里horizontal_norm_for_noise 用 测量值FL面模长 更严谨
    float horizontal_norm_for_noise = geo_mag_ref_dir_FL_vec_b_norm;
    if (horizontal_norm_for_noise <= 1.0e-6f) {
        return true;
    }
    R_mag_yaw /= (horizontal_norm_for_noise * horizontal_norm_for_noise);

    float S_mag_yaw = R_mag_yaw;
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DIM; i++) {
        S_mag_yaw += instance->temp_mat_1x9_0_data_[i] * instance->temp_mat_9x1_1_data_[i];
    }
    if (S_mag_yaw <= 1.0e-6f) {
        return false;
    }

    float mag_yaw_innovation_chi_square = (psi_residual * psi_residual) / S_mag_yaw;
    if (mag_yaw_innovation_chi_square > ALGORITHM_ESKF_MAG_YAW_ONLY_CHI_SQUARE_THRESHOLD) {
        return true;
    }

    //
    // 标量卡尔曼增益
    //
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DIM; i++) {
        instance->temp_mat_9x1_0_data_[i] = instance->temp_mat_9x1_1_data_[i] / S_mag_yaw;
    }

    //
    // 后验误差状态
    //
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DIM; i++) {
        instance->error_states_data_[i] = instance->temp_mat_9x1_0_data_[i] * psi_residual;
    }

    if (injectErrorToNominal(instance) == false) {
        return false;
    }

    if (updatePosteriorPMagYawOnly(instance, instance->temp_mat_9x1_1_data_, S_mag_yaw) == false) {
        return false;
    }

    if (clearErrorStates(instance) == false) {
        return false;
    }

    return true;
}
