#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "motor_def.h"
#include "user_def.h"

#define MOTOR_RMD_V2_X6_MAX_INSTANCE_NUM 4U
#ifndef MOTOR_RMD_V2_X6_ENABLE_CAN_ID_CONFIG
#define MOTOR_RMD_V2_X6_ENABLE_CAN_ID_CONFIG USER_RMD_V2_X6_CAN_ID_CONFIG_ENABLE
#endif

typedef enum
{
    MOTOR_RMD_V2_X6_MOTOR_ID_1 = 1,
    MOTOR_RMD_V2_X6_MOTOR_ID_2,
    MOTOR_RMD_V2_X6_MOTOR_ID_3,
    MOTOR_RMD_V2_X6_MOTOR_ID_4,
} motorRMDV2X6MotorID_e; // 最多32个，暂时不写多了

typedef enum
{
    MOTOR_RMD_V2_X6_SINGLE_BASE_ID = 0x140U,
} motorRMDV2X6TxBaseID_e; // tx时可以按单电机或group发送，这里暂时只支持单电机

typedef uint64_t (*motorRMDV2X6GetAbsTimeUs_f)(void);
typedef struct motor_rmd_v2_x6_instance motorRMDV2X6Instance_t;

typedef struct motor_rmd_v2_x6_config
{
    bspCANInstance_t *can_instance_;

    motorRMDV2X6MotorID_e motor_id_;
    float reduction_ratio_; // 减速比

    uint32_t fb_abs_angle_high_accuracy_timeout_us_; // 统一反馈超时时间

    motorRMDV2X6GetAbsTimeUs_f abs_time_us_callback_;

    const char *name_;
} motorRMDV2X6Config_t;

// 初始化电机实例，会注册CAN RX回调拷贝数据
motorRMDV2X6Instance_t *motorRMDV2X6InstanceInit(motorRMDV2X6Config_t *config);
// 设置力矩参考，Nm
bool motorRMDV2X6SetEffortRef(motorRMDV2X6Instance_t *instance, float effort_ref_Nm);
bool motorRMDV2X6SetWorkStatus(motorRMDV2X6Instance_t *instance, motorWorkStatus_e work_status);
// 只发送力矩闭环控制报文
motorStatus_e motorRMDV2X6SendEffortCommand(const motorRMDV2X6Instance_t *instance);
// 解析已到达的反馈，并在内部请求下一次高精度多圈角度反馈
motorStatus_e motorRMDV2X6UpdateFeedbackData(motorRMDV2X6Instance_t *instance);
// 获取当前缓存的统一反馈；若缓存尚未初始化或已超时，返回非 MOTOR_OK
motorStatus_e motorRMDV2X6GetFeedbackData(motorRMDV2X6Instance_t *instance, motorFeedBackData_t *data_out);

#if MOTOR_RMD_V2_X6_ENABLE_CAN_ID_CONFIG
// 独立于当前驱动实例逻辑的单电机 CAN ID 设置 utility。
// 使用前应确保总线上只连接目标电机，避免多个设备同时响应 0x300 配置报文。
motorStatus_e motorRMDV2X6SetSingleMotorCANID(bspCANInstance_t *can_instance, uint16_t can_id);
#endif


motorStatus_e motorRMDV2X6SendReadMultiRoundsAngleCommand(const motorRMDV2X6Instance_t *instance);