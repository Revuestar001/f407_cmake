#pragma once

#include "arm_math.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "quaternion.h"
#include "matrix.h"

#define ALGORITHM_ESKF_MAX_DT_S 0.01f
#define ALGORITHM_ESKF_MIN_DT_S 0.0002f

#define ALGORITHM_ESKF_IMU_SAMPLE_FREQUENCY 400.0f // imu近似采样频率，只能用于初始化！
#define ALGORITHM_ESKF_R_ACCEL_SCALE_FACTOR 10.0f // 加速度测量噪声放大系数
#define ALGORITHM_ESKF_MAG_SAMPLE_FREQUENCY 200.0f // mag近似采样频率，只能用于初始化！
#define ALGORITHM_ESKF_R_MAG_SCALE_FACTOR 20.0f // 磁力计测量噪声放大系数

#define ALGORITHM_ESKF_ACCEL_NORM_GATE_RATIO 0.15f // accel模长门限，允许相对重力参考模长的偏差比例，*100%
#define ALGORITHM_ESKF_CHI_SQUARE_1_DOF_90_PERCENT 2.706f
#define ALGORITHM_ESKF_CHI_SQUARE_1_DOF_95_PERCENT 3.841f
#define ALGORITHM_ESKF_CHI_SQUARE_1_DOF_99_PERCENT 6.635f
#define ALGORITHM_ESKF_CHI_SQUARE_1_DOF_995_PERCENT 7.879f
#define ALGORITHM_ESKF_CHI_SQUARE_3_DOF_90_PERCENT 6.251f
#define ALGORITHM_ESKF_CHI_SQUARE_3_DOF_95_PERCENT 7.815f
#define ALGORITHM_ESKF_CHI_SQUARE_3_DOF_99_PERCENT 11.345f
#define ALGORITHM_ESKF_CHI_SQUARE_3_DOF_995_PERCENT 12.838f
#define ALGORITHM_ESKF_ACCEL_CHI_SQUARE_THRESHOLD ALGORITHM_ESKF_CHI_SQUARE_3_DOF_99_PERCENT // accel卡方检验
#define ALGORITHM_ESKF_MAG_NORM_GATE_RATIO 0.15f // mag模长门限，允许相对重力参考模长的偏差比例，*100%
#define ALGORITHM_ESKF_MAG_CHI_SQUARE_THRESHOLD ALGORITHM_ESKF_CHI_SQUARE_3_DOF_99_PERCENT // mag卡方检验
#define ALGORITHM_ESKF_MAG_YAW_ONLY_CHI_SQUARE_THRESHOLD ALGORITHM_ESKF_CHI_SQUARE_1_DOF_99_PERCENT // mag yaw-only卡方检验
#define ALGORITHM_ESKF_MAG_YAW_ONLY_REF_NORM_GATE_RATIO 0.15f // mag yaw-only下参考磁场向量模长门限
#define ALGORITHM_ESKF_MAG_YAW_ONLY_MEASURE_NORM_GATE_RATIO 0.15f // mag yaw-only下测量磁场向量模长门限

typedef enum
{
    ALGORITHM_ESKF_NOMINAL_STATE_DIM = 7, // quat + bias_gyro
    ALGORITHM_ESKF_NOMINAL_STATE_QUAT_DIM = 4,
    ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM = 3,
    ALGORITHM_ESKF_ERROR_STATE_DIM = 6, // small angle error + delta bias_gyro
    ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM = 3,
    ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM = 3,
    ALGORITHM_ESKF_MEASURE_GYRO_DIM = 3,
    ALGORITHM_ESKF_MEASURE_ACCEL_DIM = 3,
    ALGORITHM_ESKF_MEASURE_MAG_DIM = 3,
} algorithmESKFDims_e;

typedef enum
{
    ALGORITHM_ESKF_ENU_FLU = 0,
} algorithmESKFFrame_e;

typedef enum
{
    ALGORITHM_ESKF_MAG_MEASUREMENT_RAW_VECTOR = 0,
    ALGORITHM_ESKF_MAG_MEASUREMENT_NORMALIZED_VECTOR,
} algorithmESKFMagMeasurementMode_e;

typedef struct eskf_init_params
{
    mathQuaternion_t quat_init_;
    float gyro_bias_init_[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM];

    // 初始化状态误差协方差，姿态误差方差、gyro bias方差
    float angle_error_variance_[ALGORITHM_ESKF_ERROR_STATE_SMALL_ANGLE_ERROR_DIM];
    float delta_bias_gyro_variance_[ALGORITHM_ESKF_ERROR_STATE_DELTA_GYRO_BIAS_DIM];

    // 重力参考向量,比力，N系下
    float gravity_ref_n_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    // 地磁参考向量，N系下
    // raw模式下使用带模长参考，normalized模式下会在init里归一化成方向向量
    float geo_mag_ref_dir_n_[ALGORITHM_ESKF_MEASURE_MAG_DIM];
} algorithmESKFInitParams_t;

typedef struct eskf_params
{
    algorithmESKFFrame_e frame_;
    algorithmESKFMagMeasurementMode_e mag_measurement_mode_;

    algorithmESKFInitParams_t init_params_;

    // 过程噪声Q参数
    float gyro_noise_rads_sqrt_hz_; // gyro白噪声密度，rad/s/sqrt(Hz)，对应小角度误差传播
    float gyro_random_walk_rads2_sqrt_hz_; // gyro随机游走，rad/s2/sqrt(Hz)，对应delta bias_gyro 传播

    // 测量/观测噪声R参数
    float accel_noise_ms2_sqrt_hz_; // accel白噪声密度，m/s2/sqrt(Hz)，加速度计测量噪声
    float mag_noise_ut_sqrt_hz_; // mag白噪声密度，ut/sqrt(Hz)，磁力计测量噪声
} algorithmESKFParams_t;

typedef struct eskf
{
    // 名义状态
    mathQuaternion_t nominal_state_quat_;
    mathMatrix_t nomial_state_bias_gyro_; 
    float nomial_state_bias_gyro_data_[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM];

    // 误差状态
    mathMatrix_t error_states_;
    float error_states_data_[ALGORITHM_ESKF_ERROR_STATE_DIM];

    // 状态误差协方差矩阵和噪声协方差矩阵
    mathMatrix_t P_;
    mathMatrix_t Q_; // 离散形式
    mathMatrix_t R_accel_; // 离散形式
    mathMatrix_t R_mag_; // 离散形式
    float P_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float Q_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float R_accel_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float R_mag_data_[ALGORITHM_ESKF_MEASURE_MAG_DIM * ALGORITHM_ESKF_MEASURE_MAG_DIM];

    // 误差状态一阶雅可比矩阵F、误差状态转移矩阵PHI、噪声输入矩阵G、测量一阶雅可比矩阵H、卡尔曼增益矩阵K、创新协方差矩阵S
    mathMatrix_t F_; 
    mathMatrix_t PHI_; // 离散形式
    mathMatrix_t G_;
    mathMatrix_t H_accel_;
    mathMatrix_t H_mag_;
    mathMatrix_t S_accel_;
    mathMatrix_t S_mag_;
    mathMatrix_t K_accel_;
    mathMatrix_t K_mag_;
    float F_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float PHI_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float G_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float H_accel_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float H_mag_data_[ALGORITHM_ESKF_MEASURE_MAG_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float S_accel_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float S_mag_data_[ALGORITHM_ESKF_MEASURE_MAG_DIM * ALGORITHM_ESKF_MEASURE_MAG_DIM];
    float K_accel_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float K_mag_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_MAG_DIM];

    // 临时temp矩阵的数组，降低任务栈压力
    float temp_mat_9x9_0_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x9_1_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x9_2_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x9_3_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x9_4_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x9_5_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x9_6_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x3_0_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_9x3_1_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x9_0_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_3x3_0_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x3_1_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x3_2_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x3_3_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x3_4_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x3_5_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_3x1_0_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    float temp_mat_3x1_1_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    float temp_mat_3x1_2_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    float temp_mat_3x1_3_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    float temp_mat_3x1_4_data_[ALGORITHM_ESKF_MEASURE_ACCEL_DIM * 1U];
    float temp_mat_1x3_0_data_[1U * ALGORITHM_ESKF_MEASURE_ACCEL_DIM];
    float temp_mat_1x9_0_data_[1U * ALGORITHM_ESKF_ERROR_STATE_DIM];
    float temp_mat_9x1_0_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * 1U];
    float temp_mat_9x1_1_data_[ALGORITHM_ESKF_ERROR_STATE_DIM * 1U];
    
    // 重力参考向量,比力，N系下
    mathMatrix_t gravity_ref_n_;
    // 地磁参考方向，N系下
    mathMatrix_t geo_mag_ref_dir_n_;
    float geo_mag_ref_norm_;

    float dt_;

    // 参数和初始化参数
    algorithmESKFParams_t params_;

    bool is_initialized_;
} algorithmESKF_t;

bool algorithmESKFInit(algorithmESKF_t *instance, algorithmESKFParams_t *params);
// 使用gyro测量值进行预测
// measurement必须是不包含在线bias校正的，否则eskf这里会重复去偏bias
bool algorithmESKFGyroPredict(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_NOMINAL_STATE_BIAS_GYRO_DIM], float dt);
// 使用accel测量值(只包含offline bias + scale)更新
// dt当前保留接口，按当前同步IMU语义暂未使用
bool algorithmESKFAccelUpdate(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_MEASURE_ACCEL_DIM], float dt);
// 使用mag测量值(只包含hard-iron/soft-iron)更新
bool algorithmESKFMagUpdate(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_MEASURE_MAG_DIM], float dt);
// 使用mag测量值(只包含hard-iron/soft-iron)进行yaw-only更新
bool algorithmESKFMagUpdateYawOnly(algorithmESKF_t *instance, float measurement[ALGORITHM_ESKF_MEASURE_MAG_DIM], float dt);
