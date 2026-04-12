#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "motor_def.h"

#define MOTOR_DJI_M3508_MAX_INSTANCE_NUM 4U

typedef enum
{
    MOTOR_DJI_M3508_MOTOR_ID_1 = 1,
    MOTOR_DJI_M3508_MOTOR_ID_2,
    MOTOR_DJI_M3508_MOTOR_ID_3,
    MOTOR_DJI_M3508_MOTOR_ID_4,
    MOTOR_DJI_M3508_MOTOR_ID_5,
    MOTOR_DJI_M3508_MOTOR_ID_6,
    MOTOR_DJI_M3508_MOTOR_ID_7,
    MOTOR_DJI_M3508_MOTOR_ID_8,
} motorDJIM3508MotorID_e;

typedef enum
{
    MOTOR_DJI_M3508_GROUP1_BASE_ID = 0x200U,
    MOTOR_DJI_M3508_GROUP2_BASE_ID = 0x1FFU,
} motorDJIM3508TxGroupBaseID_e; // tx时只能按group发送而不是按单电机

typedef uint64_t (*motorDJIM3508GetAbsTimeUs_f)(void);
typedef struct motor_dji_m3508_instance motorDJIM3508Instance_t;

typedef struct motor_dji_m3508_config
{
    bspCANInstance_t *can_instance_;

    motorDJIM3508MotorID_e motor_id_;
    float reduction_ratio_; // 减速比

    motorDJIM3508GetAbsTimeUs_f abs_time_us_callback_;

    const char *name_;
} motorDJIM3508Config_t;

// 初始化电机实例，会注册CAN RX回调拷贝数据
motorDJIM3508Instance_t *motorDJIM3508InstanceInit(motorDJIM3508Config_t *config);
// 设置参考电流值，安培
bool motorDJIM3508SetCurrentRef(motorDJIM3508Instance_t *instance, float current_ref_A);
// 设置启停
bool motorDJIM3508SetWorkStatus(motorDJIM3508Instance_t *instance, motorWorkStatus_e work_status);
// 以单个电机实例为锚点，按所在group发送can报文，注意暂时没有阻塞时间
motorStatus_e motorDJIM3508GroupSendByMotorInstance(const motorDJIM3508Instance_t *anchor_instance);
// 解析并更新反馈数据，带绝对us时间戳
motorStatus_e motorDJIM3508UpdateFeedbackData(motorDJIM3508Instance_t *instance);
// 获取反馈数据
motorStatus_e motorDJIM3508GetFeedbackData(motorDJIM3508Instance_t *instance, motorFeedBackData_t *data_out);
// 获取该电机所在的group信息
bool motorDJIM3508GetCommitGroup(motorDJIM3508Instance_t *instance, const void **commit_bus, uint32_t *group_id);