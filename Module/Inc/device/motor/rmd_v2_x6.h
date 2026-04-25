#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "motor_def.h"

#define MOTOR_RMD_V2_X6_MAX_INSTANCE_NUM 4U

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
} motorRMDV2X6TxBaseID_e; // tx时可以按单电机或group发送

typedef uint64_t (*motorRMDV2X6GetAbsTimeUs_f)(void);
typedef struct motor_rmd_v2_x6_instance motorRMDV2X6Instance_t;

typedef struct motor_rmd_v2_x6_config
{
    bspCANInstance_t *can_instance_;

    motorRMDV2X6MotorID_e motor_id_;
    float reduction_ratio_; // 减速比

    motorRMDV2X6GetAbsTimeUs_f abs_time_us_callback_;

    const char *name_;
} motorRMDV2X6Config_t;

// 初始化电机实例，会注册CAN RX回调拷贝数据
motorRMDV2X6Instance_t *motorRMDV2X6InstanceInit(motorRMDV2X6Config_t *config);
