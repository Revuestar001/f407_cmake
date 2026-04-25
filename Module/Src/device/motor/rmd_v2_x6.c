#include <stdint.h>
#include <string.h>

#include "motor_def.h"
#include "general_math.h"
#include "rmd_v2_x6.h"

#define MOTOR_RMD_V2_X6_CAN_RX_DLC 8U
#define MOTOR_RMD_V2_X6_RX_BASE_ID 0x240U

#define MOTOR_RMD_V2_X6_PEAK_TORQUE_NM 7.0f

typedef enum
{
    MOTOR_RMD_V2_X6_TX_CMD_READ_MULTI_ROUNDS_ANGLE = 0x92U,
    MOTOR_RMD_V2_X6_TX_CMD_TORQUE_LOOP = 0xA1U, // 力矩闭环控制
} motorRMDV2X6TxCommand_e;

typedef enum
{
    MOTOR_RMD_V2_X6_RX_CMD_READ_MULTI_ROUNDS_ANGLE = 0x92U,
    MOTOR_RMD_V2_X6_RX_CMD_TORQUE_LOOP = 0xA1U,
} motorRMDV2X6RxCommand_e;

typedef struct motor_rmd_v2_x6_instance 
{
    bspCANInstance_t *can_instance_;
    
    motorRMDV2X6MotorID_e motor_id_;
    float reduction_ratio_;
    motorRMDV2X6TxBaseID_e tx_base_id_;
    uint32_t rx_id_;

    float effort_ref_Nm_; // 转矩控制

    uint8_t rx_data_[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    uint64_t rx_data_timestamp_us_;

    uint8_t fb_data_raw_[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    uint16_t fb_data_raw_last_ecd_; // 上一次编码器值
    int32_t total_ecd_; // 总编码器值，带方向
    bool angle_is_initialize_; // 用于为 总编码器值 设置0点
    uint64_t fb_data_raw_timestamp_us_;
    volatile bool fb_data_raw_is_new_; // 表示反馈数据是否是最新的
    motorFeedBackData_t fb_data_;

    motorWorkStatus_e work_status_;

    motorRMDV2X6GetAbsTimeUs_f abs_time_us_callback_;

    const char *name_;
} motorRMDV2X6Instance_t;

static uint8_t x6_memory_index_ = 0U;
static motorRMDV2X6Instance_t x6_instance_memory_[MOTOR_RMD_V2_X6_MAX_INSTANCE_NUM] = {0};

static bool checkMotorIDValid(bspCANInstance_t *can_instance, motorRMDV2X6MotorID_e motor_id)
{
    if (can_instance == NULL) {
        return false;
    }

    if (motor_id < MOTOR_RMD_V2_X6_MOTOR_ID_1 || motor_id > MOTOR_RMD_V2_X6_MOTOR_ID_4) {
        return false;
    }

    for (size_t i = 0; i < x6_memory_index_; i ++) {
        if (can_instance == x6_instance_memory_[i].can_instance_ && motor_id == x6_instance_memory_[i].motor_id_) {
            return false;
        }
    }

    return true;
}

// bsp_can 接收回调函数
static void motorFeedbackCallback(void *owner, const bspCANMessage_t *rx_message)
{
    if (owner == NULL || rx_message == NULL) {
        return;
    }

    motorRMDV2X6Instance_t *instance = (motorRMDV2X6Instance_t *)owner;

    if (instance->abs_time_us_callback_ == NULL || rx_message->message_header_.message_dlc_ != MOTOR_RMD_V2_X6_CAN_RX_DLC) {
        return;
    }

    memcpy(instance->rx_data_, rx_message->message_data_, MOTOR_RMD_V2_X6_CAN_RX_DLC);
    instance->rx_data_timestamp_us_ = instance->abs_time_us_callback_();
}

static bool registerMotorFeedbackCallback(motorRMDV2X6Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    bspCANRxRoute_t rx_route;
    rx_route.route_id_ = instance->rx_id_;
    rx_route.route_ide_ = 0U;
    rx_route.route_id_mask_ = 0U; // 用不上，暂时随便给
    rx_route.route_owner_ = instance;
    rx_route.route_rx_callback_ = motorFeedbackCallback;

    bspCANRxCallbackRegister(instance->can_instance_, rx_route);

    return true;
}

motorRMDV2X6Instance_t *motorRMDV2X6InstanceInit(motorRMDV2X6Config_t *config)
{
    if (config == NULL || config->abs_time_us_callback_ == NULL || config->reduction_ratio_ <= 0.0f) {
        return NULL;
    }

    if (x6_memory_index_ >= MOTOR_RMD_V2_X6_MAX_INSTANCE_NUM) {
        return NULL;
    }

    // x6 ID不允许重复
    if (checkMotorIDValid(config->can_instance_, config->motor_id_) == false) {
        return NULL;
    }

    motorRMDV2X6Instance_t *instance = &x6_instance_memory_[x6_memory_index_];
    memset(instance, 0, sizeof(motorRMDV2X6Instance_t));
    instance->can_instance_ = config->can_instance_;
    instance->motor_id_ = config->motor_id_;
    instance->reduction_ratio_ = config->reduction_ratio_;
    instance->rx_id_ = MOTOR_RMD_V2_X6_RX_BASE_ID + config->motor_id_;

    instance->angle_is_initialize_ = false;
    instance->fb_data_raw_is_new_ = false;
    instance->work_status_ = MOTOR_WORK_STATUS_DISABLE; // 默认关闭，比较安全
    instance->abs_time_us_callback_ = config->abs_time_us_callback_;
    instance->name_ = config->name_;

    if (registerMotorFeedbackCallback(instance) == false) {
        return NULL;
    }

    x6_memory_index_ ++;

    return instance;
}

bool motorRMDV2X6SetEffortRef(motorRMDV2X6Instance_t *instance, float effort_ref_Nm)
{
    if (instance == NULL) {
        return false;
    }

    float effort_ref_clamped_Nm;
    mathClampf(effort_ref_Nm, -MOTOR_RMD_V2_X6_PEAK_TORQUE_NM, MOTOR_RMD_V2_X6_PEAK_TORQUE_NM, &effort_ref_clamped_Nm);

    instance->effort_ref_Nm_ = effort_ref_clamped_Nm;

    return true;
}

bool motorRMDV2X6SetWorkStatus(motorRMDV2X6Instance_t *instance, motorWorkStatus_e work_status)
{
    if (instance == NULL) {
        return false;
    }

    instance->work_status_ = work_status;

    return true;
}

motorStatus_e motorRMDV2X6SendCommand(const motorRMDV2X6Instance_t *instance)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    bspCANMessage_t torque_message;
    // if (buildCANTxMessage(&anchor_instance->tx_group_base_id_, tx_data, &torque_message) == false) {
    //     return MOTOR_ERROR;
    // }

    // if (bspCANTransmit(anchor_instance->can_instance_, &tx_message) != BSP_CAN_OK) {
    //     return MOTOR_TX_FAILURE;
    // }
    
    return  MOTOR_OK;
}