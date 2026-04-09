#include "arm_math.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "error_state_kalman_filter.h"
#include "matrix.h"
#include "quaternion.h"
#include "vector3.h"

static bool computeQDiscrete(algorithmESKF_t *instance, float dt)
{
    if (instance == NULL) {
        return false;
    }

    // 对角线元素
    float val_angle_angle = instance->params_.gyro_noise_rads_sqrt_hz_ * instance->params_.gyro_noise_rads_sqrt_hz_ * dt + 
                            instance->params_.gyro_random_walk_rads2_sqrt_hz_ * instance->params_.gyro_random_walk_rads2_sqrt_hz_ * dt * dt * dt / 3.0f;
    float val_gyro_gyro = instance->params_.gyro_random_walk_rads2_sqrt_hz_ * instance->params_.gyro_random_walk_rads2_sqrt_hz_ * dt;
    float val_accel_accel = instance->params_.accel_random_walk_ms3_sqrt_hz_ * instance->params_.accel_random_walk_ms3_sqrt_hz_ * dt;
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
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM + ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM + ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, 
                                val_accel_accel);
    mathMatrixSetBlockDiagScalar(&instance->Q_, 
                                0U, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, 
                                val_angle_gyro);
    mathMatrixSetBlockDiagScalar(&instance->Q_, 
                                ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, 
                                0U, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, 
                                ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, 
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

    // 注入误差，计算后验名义bias accel
    for (size_t i = 0; i < ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM; i ++) {
        instance->nomial_state_bias_accel_data_[i] += instance->error_states_data_[ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM + ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM + i];
    }

    return true;
}

static bool updatePosteriorPAccel(algorithmESKF_t *instance)
{
    if (instance == NULL || instance->is_initialized_ == false) {
        return false;
    }

    // K * H
    mathMatrix_t K_H_accel;
    float K_H_accel_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&K_H_accel, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, K_H_accel_data);
    mathMatrixMult(&instance->K_accel_, &instance->H_accel_, &K_H_accel);
    // (I - K * H)
    mathMatrix_t identity;
    float identity_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&identity, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, identity_data);
    mathMatrixSetIdentity(&identity);
    mathMatrix_t I_K_H_accel;
    float I_K_H_accel_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&I_K_H_accel, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, I_K_H_accel_data);
    mathMatrixSub(&identity, &K_H_accel, &I_K_H_accel);
    // (I - K * H)^T
    mathMatrix_t I_K_H_accel_T;
    float I_K_H_accel_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&I_K_H_accel_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, I_K_H_accel_T_data);
    mathMatrixTranspose(&I_K_H_accel, &I_K_H_accel_T);
    // (I - K * H) * P_priori
    mathMatrix_t I_K_H_accel_P_priori;
    float I_K_H_accel_P_priori_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&I_K_H_accel_P_priori, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, I_K_H_accel_P_priori_data);
    mathMatrixMult(&I_K_H_accel, &instance->P_, &I_K_H_accel_P_priori);
    // (I - K * H) * P_priori * (I - K * H)^T
    mathMatrix_t I_K_H_accel_P_priori_I_K_H_accel_T;
    float I_K_H_accel_P_priori_I_K_H_accel_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&I_K_H_accel_P_priori_I_K_H_accel_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, I_K_H_accel_P_priori_I_K_H_accel_T_data);
    mathMatrixMult(&I_K_H_accel_P_priori, &I_K_H_accel_T, &I_K_H_accel_P_priori_I_K_H_accel_T);
    // K^T
    mathMatrix_t K_accel_T;
    float K_accel_T_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&K_accel_T, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, K_accel_T_data);
    mathMatrixTranspose(&instance->K_accel_, &K_accel_T);
    // K * R
    mathMatrix_t K_R_accel;
    float K_R_accel_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    mathMatrixInit(&K_R_accel, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, K_R_accel_data);
    mathMatrixMult(&instance->K_accel_, &instance->R_accel_, &K_R_accel);
    // K * R * K^T
    mathMatrix_t K_R_K_accel_T;
    float K_R_K_accel_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&K_R_K_accel_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, K_R_K_accel_T_data);
    mathMatrixMult(&K_R_accel, &K_accel_T, &K_R_K_accel_T);
    // P_posterior = (I - K * H) * P_priori * (I - K * H)^T + K * R * K^T
    mathMatrixAdd(&I_K_H_accel_P_priori_I_K_H_accel_T, &K_R_K_accel_T, &instance->P_);
    mathMatrixSetSymmetricInPlace(&instance->P_);

    return true;
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
    mathMatrixInit(&instance->nomial_state_bias_accel_, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_ACCEL_DIM, 1U, instance->nomial_state_bias_accel_data_);
    
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
    memcpy(instance->nomial_state_bias_accel_data_, params->init_params_.accel_bias_init_, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_ACCEL_DIM * sizeof(float));

    // 设置初始状态误差协方差矩阵
    float p_diag_init[ALGORITHM_ESKF_ERROR_STATE_DIM];
    memcpy(p_diag_init, params->init_params_.angle_error_variance_, ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM * sizeof(float));
    memcpy(p_diag_init + ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM, params->init_params_.delta_bias_gyro_variance_, ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM * sizeof(float));
    memcpy(p_diag_init + ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM + ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, params->init_params_.delta_bias_accel_variance_, ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM * sizeof(float));
    if (mathMatrixSetDiag(&instance->P_, p_diag_init, ALGORITHM_ESKF_ERROR_STATE_DIM) == false) {
        return false;
    }

    // 设置测量噪声协方差矩阵R,这里是工程近似,元素值val = (连续时间噪声密度^2 * 采样频率f * 0.5) * 放大系数
    float r_accel_init_val = params->accel_noise_ms2_sqrt_hz_ * params->accel_noise_ms2_sqrt_hz_ * ALGORITHM_ESKF_IMU_SAMPLE_FREQUENCY * 0.5f * ALGORITHM_ESKF_R_ACCEL_SCALE_FACTOR;
    float r_mag_init_val = params->mag_noise_ut_sqrt_hz_ * params->mag_noise_ut_sqrt_hz_ * ALGORITHM_ESKF_MAG_SAMPLE_FREQUENCY * 0.5f * ALGORITHM_ESKF_R_MAG_SCALE_FACTOR;
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

    // 存储重力(比力)参考和地磁方向参考
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

    // 计算名义状态传播后的 bias,这里其实就是保持不变，所以就不做操作了
    // float gyro_bias_priori[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM];
    // float accel_bias_priori[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_ACCEL_DIM];
    // memcpy(gyro_bias_priori, instance->nomial_state_bias_gyro_data_, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM * sizeof(float));
    // memcpy(accel_bias_priori, instance->nomial_state_bias_accel_data_, ALGORITHM_ESKF_NOMINAL_STATE_BIAS_ACCEL_DIM * sizeof(float));

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
    // 一阶近似
    mathMatrix_t F_dt;
    float F_dt_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&F_dt, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, F_dt_data);
    mathMatrixScale(&instance->F_, dt, &F_dt);
    mathMatrix_t identity;
    float identity_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&identity, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, identity_data);
    mathMatrixSetIdentity(&identity);
    mathMatrixAdd(&identity, &F_dt, &instance->PHI_);

    //
    // 状态误差协方差矩阵P传播
    //
    // 当前先验P = PHI * 当前后验P * PHI^T + Q
    mathMatrix_t PHI_P_posterior_PHI_T;
    float PHI_P_posterior_PHI_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&PHI_P_posterior_PHI_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, PHI_P_posterior_PHI_T_data);
    mathMatrix_t PHI_T;
    float PHI_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&PHI_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, PHI_T_data);
    mathMatrixTranspose(&instance->PHI_, &PHI_T);
    mathMatrix_t PHI_P_posterior;
    float PHI_P_posterior_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    mathMatrixInit(&PHI_P_posterior, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_ERROR_STATE_DIM, PHI_P_posterior_data);
    mathMatrixMult(&instance->PHI_, &instance->P_, &PHI_P_posterior);
    mathMatrixMult(&PHI_P_posterior, &PHI_T, &PHI_P_posterior_PHI_T);
    mathMatrixAdd(&PHI_P_posterior_PHI_T, &instance->Q_, &instance->P_);
    // 数值对称化
    mathMatrixSetSymmetricInPlace(&instance->P_);

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
    // 计算先验姿态旋转矩阵
    mathMatrix_t rotate_matrix_b_to_n;
    float rotate_matrix_b_to_n_data[3U * 3U];
    mathMatrixInit(&rotate_matrix_b_to_n, 3U, 3U, rotate_matrix_b_to_n_data);
    mathQuaternionToRotationMatrix(&instance->nominal_state_quat_, rotate_matrix_b_to_n_data);
    mathMatrix_t rotate_matrix_n_to_b;
    float rotate_matrix_n_to_b_data[3U * 3U];
    mathMatrixInit(&rotate_matrix_n_to_b, 3U, 3U, rotate_matrix_n_to_b_data);
    mathMatrixTranspose(&rotate_matrix_b_to_n, &rotate_matrix_n_to_b);

    // 计算预测加速度计测量值
    // accel_measure_predict = C_n_to_b * gravity_vec_n + bias_accel_priori
    mathMatrix_t gravity_vec_b;
    float gravity_vec_b_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    mathMatrixInit(&gravity_vec_b, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, 1U, gravity_vec_b_data);
    mathMatrixMult(&rotate_matrix_n_to_b, &instance->gravity_ref_n_, &gravity_vec_b);
    mathMatrix_t accel_measure_predict;
    float accel_measure_predict_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    mathMatrixInit(&accel_measure_predict, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, 1U, accel_measure_predict_data);
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U; i ++) {
        accel_measure_predict_data[i] = gravity_vec_b_data[i] + instance->nomial_state_bias_accel_data_[i];
    }

    // 
    // 计算加速度计测量残差
    //
    // accel_residual = accel_measure - accel_measure_predict
    mathMatrix_t accel_residual;
    float accel_residual_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    mathMatrixInit(&accel_residual, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, 1U, accel_residual_data);
    for (size_t i = 0; i < ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U; i ++) {
        accel_residual_data[i] = measurement[i] - accel_measure_predict_data[i];
    }

    //
    // 计算accel观测/测量一阶雅可比矩阵H
    //
    // 请注意，这里计算H时,gravity_vec_b_skew的负号去掉了才正常，与推导不一致,基本上认为是推导错误！
    if (mathMatrixSetZero(&instance->H_accel_) == false) {
        return false;
    }
    mathVector3_t gravity_vec_b_vec3;
    memcpy(gravity_vec_b_vec3.v_, gravity_vec_b_data, sizeof(gravity_vec_b_data));
    mathMatrix_t gravity_vec_b_skew;
    float gravity_vec_b_skew_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    mathMatrixInit(&gravity_vec_b_skew, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, gravity_vec_b_skew_data);
    mathVec3BuildSkewSymmetricMatrix(&gravity_vec_b_vec3, gravity_vec_b_skew_data);
    mathMatrixSetBlockByMatrix(&instance->H_accel_, 0U, 0U, &gravity_vec_b_skew);
    mathMatrixSetBlockIdentity(&instance->H_accel_, 0U, ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM + ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM, ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM, ALGORITHM_ESKF_ERROR_STATE_DELTA_ACCEL_BIAS_DIM);

    // 
    // 计算创新协方差矩阵S
    //
    // S_accel = H_accel * P_priori * H_accel^T + R_accel
    // H_accel^T
    mathMatrix_t H_accel_T;
    float H_accel_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    mathMatrixInit(&H_accel_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, H_accel_T_data);
    mathMatrixTranspose(&instance->H_accel_, &H_accel_T);
    // P_priori * H_accel^T
    mathMatrix_t P_priori_H_accel_T;
    float P_priori_H_accel_T_data[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    mathMatrixInit(&P_priori_H_accel_T, ALGORITHM_ESKF_ERROR_STATE_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, P_priori_H_accel_T_data);
    mathMatrixMult(&instance->P_, &H_accel_T, &P_priori_H_accel_T);
    // H_accel * P_priori * H_accel^T
    mathMatrix_t H_accel_P_priori_H_accel_T;
    float H_accel_P_priori_H_accel_T_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    mathMatrixInit(&H_accel_P_priori_H_accel_T, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, H_accel_P_priori_H_accel_T_data);
    mathMatrixMult(&instance->H_accel_, &P_priori_H_accel_T, &H_accel_P_priori_H_accel_T);
    mathMatrixAdd(&H_accel_P_priori_H_accel_T, &instance->R_accel_, &instance->S_accel_);

    //
    // 计算卡尔曼增益K
    //
    // K_accel = P_priori * H_accel^T * S_accel^-1
    mathMatrix_t S_accel_inv;
    float S_accel_inv_data[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    mathMatrixInit(&S_accel_inv, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, ALGORITHM_ESKF_MEASURE_ACCEL_DIM, S_accel_inv_data);
    if (mathMatrixInverse(&instance->S_accel_, &S_accel_inv) != ARM_MATH_SUCCESS) {
        return false;
    }
    // 创新卡方门限不通过，本轮不使用accel更新，但不视为滤波失败
    float accel_innovation_chi_square = 0.0f;
    if (computeAccelInnovationChiSquare(accel_residual_data, &S_accel_inv, &accel_innovation_chi_square) == false) {
        return false;
    }
    if (accel_innovation_chi_square > ALGORITHM_ESKF_ACCEL_CHI_SQUARE_THRESHOLD) {
        return true;
    }
    mathMatrixMult(&P_priori_H_accel_T, &S_accel_inv, &instance->K_accel_);

    //
    // 计算误差状态后验估计error_states,9x1
    //
    // error_states_posterior = K_accel * accel_residual
    mathMatrixMult(&instance->K_accel_, &accel_residual, &instance->error_states_);

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
