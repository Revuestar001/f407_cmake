#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bsp_can.h"
#include "bsp_critical.h"
#include "motor_def.h"
#include "dji_m3508.h"
#include "general_math.h"

#define MOTOR_DJI_M3508_MOTOR_NUM_ONE_GROUP 4U
#define MOTOR_DJI_M3508_CAN_TX_DLC 8U
#define MOTOR_DJI_M3508_CAN_RX_DLC 8U

#define MOTOR_DJI_M3508_RX_BASE_ID 0x200U

#define MOTOR_DJI_M3508_CTRL_CURRENT_MAX_A 20.0f
#define MOTOR_DJI_M3508_CTRL_CURRENT_MIN_A -20.0f
#define MOTOR_DJI_M3508_CTRL_CURRENT_MAX_INT16 16384
#define MOTOR_DJI_M3508_CTRL_CURRENT_MIN_INT16 -16384

#define MOTOR_DJI_M3508_FEEDBACK_ANGLE_MAX_DEG 360.0f
#define MOTOR_DJI_M3508_FEEDBACK_ANGLE_MIN_DEG 0.0f
#define MOTOR_DJI_M3508_FEEDBACK_ANGLE_MAX_UINT16 8191U
#define MOTOR_DJI_M3508_FEEDBACK_ANGLE_MIN_UINT16 0U

// 编码器值
#define MOTOR_DJI_M3508_ECD_RANGE (MOTOR_DJI_M3508_FEEDBACK_ANGLE_MAX_UINT16 + 1U)
#define MOTOR_DJI_M3508_ECD_HALF_RANGE (MOTOR_DJI_M3508_ECD_RANGE / 2U)
#define MOTOR_DJI_M3508_ECD_TO_RAD (6.28318530717958647692f / (float)MOTOR_DJI_M3508_ECD_RANGE)

typedef struct motor_dji_m3508_instance
{
    bspCANInstance_t *can_instance_;
    
    motorDJIM3508MotorID_e motor_id_;
    float reduction_ratio_;
    motorDJIM3508TxGroupBaseID_e tx_group_base_id_;
    uint8_t tx_slot_; // [0,3]，表示该电机在tx是在报文中占的位置
    motorDJIM3508Instance_t *tx_group_head_; // 链表头
    motorDJIM3508Instance_t *tx_group_next_;
    uint32_t rx_id_;

    float current_ref_A_;

    uint8_t fb_data_raw_[MOTOR_DJI_M3508_CAN_RX_DLC];
    uint16_t fb_data_raw_last_ecd_; // 上一次编码器值
    int32_t total_ecd_; // 总编码器值，带方向
    bool angle_is_initialize_; // 用于为 总编码器值 设置0点
    uint64_t fb_data_raw_timestamp_us_;
    volatile bool fb_data_raw_is_new_; // 表示反馈数据是否是最新的
    motorFeedBackData_t fb_data_;

    motorWorkStatus_e work_status_;

    motorDJIM3508GetAbsTimeUs_f abs_time_us_callback_;

    const char *name_;
} motorDJIM3508Instance_t;

static uint8_t m3508_memory_index_ = 0;
static motorDJIM3508Instance_t m3508_instance_memory_[MOTOR_DJI_M3508_MAX_INSTANCE_NUM] = {0};

static bool checkMotorIDValid(bspCANInstance_t *can_instance, motorDJIM3508MotorID_e motor_id)
{
    if (can_instance == NULL) {
        return false;
    }

    if (motor_id < MOTOR_DJI_M3508_MOTOR_ID_1 || motor_id > MOTOR_DJI_M3508_MOTOR_ID_8) {
        return false;
    }

    for (size_t i = 0; i < m3508_memory_index_; i ++) {
        if (can_instance == m3508_instance_memory_[i].can_instance_ && motor_id == m3508_instance_memory_[i].motor_id_) {
            return false;
        }
    }

    return true;
}

static bool allocateTxGroupBaseID(motorDJIM3508MotorID_e motor_id, motorDJIM3508TxGroupBaseID_e *group_base_id)
{
    if (group_base_id == NULL) {
        return false;
    }

    if (motor_id < MOTOR_DJI_M3508_MOTOR_ID_1 || motor_id > MOTOR_DJI_M3508_MOTOR_ID_8) {
        return false;
    }

    switch (motor_id) {
        case MOTOR_DJI_M3508_MOTOR_ID_1:
        case MOTOR_DJI_M3508_MOTOR_ID_2:
        case MOTOR_DJI_M3508_MOTOR_ID_3:
        case MOTOR_DJI_M3508_MOTOR_ID_4:
            *group_base_id = MOTOR_DJI_M3508_GROUP1_BASE_ID;
            break;
        case MOTOR_DJI_M3508_MOTOR_ID_5:
        case MOTOR_DJI_M3508_MOTOR_ID_6:
        case MOTOR_DJI_M3508_MOTOR_ID_7:
        case MOTOR_DJI_M3508_MOTOR_ID_8:
            *group_base_id = MOTOR_DJI_M3508_GROUP2_BASE_ID;
            break;
        default:
            return false;
    }

    return true;
}

// 插入同一个group内的链表
static bool allocateTxGroup(motorDJIM3508Instance_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    for (size_t i = 0; i < m3508_memory_index_; i ++) {
        if (instance->can_instance_ == m3508_instance_memory_[i].can_instance_ && instance->tx_group_base_id_ == m3508_instance_memory_[i].tx_group_base_id_) {
            // 同一can总线上已存在至少一个同group的电机
            motorDJIM3508Instance_t *matched = &m3508_instance_memory_[i];
            motorDJIM3508Instance_t *head = matched->tx_group_head_;
            instance->tx_group_next_ = head->tx_group_next_;
            head->tx_group_next_ = instance;
            instance->tx_group_head_ = head;

            return true;
        }
    }

    // 同一can总线上没有同一group电机，说明自己是head
    instance->tx_group_head_ = instance;
    
    return true;
}

// bsp_can 接收回调函数
static void motorFeedbackCallback(void *owner, const bspCANMessage_t *rx_message)
{
    if (owner == NULL || rx_message == NULL) {
        return;
    }

    motorDJIM3508Instance_t *instance = (motorDJIM3508Instance_t *)owner;

    if (instance->abs_time_us_callback_ == NULL || rx_message->message_header_.message_dlc_ != MOTOR_DJI_M3508_CAN_RX_DLC) {
        return;
    }

    memcpy(instance->fb_data_raw_, rx_message->message_data_, MOTOR_DJI_M3508_CAN_RX_DLC);
    instance->fb_data_raw_timestamp_us_ = instance->abs_time_us_callback_();
    // 新数据到来，解析并更新后清除标志
    instance->fb_data_raw_is_new_ = true;
}

static bool registerMotorFeedbackCallback(motorDJIM3508Instance_t *instance)
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

static bool convertRefFloatToInt16(float ref_float, int16_t *ref_int16)
{
    if (ref_int16 == NULL) {
        return false;
    }

    *ref_int16 = (int16_t)((ref_float / MOTOR_DJI_M3508_CTRL_CURRENT_MAX_A) * (float)MOTOR_DJI_M3508_CTRL_CURRENT_MAX_INT16);

    return true;
}

static bool buildCANTxMessage(const motorDJIM3508TxGroupBaseID_e *tx_group_base_id_, const int16_t *tx_data, bspCANMessage_t *tx_message)
{
    if (tx_group_base_id_ == NULL || tx_data == NULL || tx_message == NULL) {
        return false;
    }

    tx_message->message_header_.message_id_ = *tx_group_base_id_;
    tx_message->message_header_.message_ide_ = 0U;
    tx_message->message_header_.message_rtr_ = 0U;
    tx_message->message_header_.message_dlc_ = MOTOR_DJI_M3508_CAN_TX_DLC;
    for (size_t i = 0; i < MOTOR_DJI_M3508_MOTOR_NUM_ONE_GROUP; i ++) {
        tx_message->message_data_[2U * i] = (uint8_t)((uint16_t)tx_data[i] >> 8); // DJI M3508协议规定，高8位在前
        tx_message->message_data_[2U * i + 1U] = (uint8_t)((uint16_t)tx_data[i]);
    }

    return true;
}

// 更新多圈总角度
static bool updateTotalAngle(motorDJIM3508Instance_t *instance, uint16_t angle_raw_ecd)
{
    if (instance == NULL) {
        return false;
    }

    if (instance->angle_is_initialize_ == false) {
        // 还未设置总角度0点
        instance->fb_data_raw_last_ecd_ = angle_raw_ecd;
        instance->total_ecd_ = 0;
        instance->angle_is_initialize_ = true;      
        instance->fb_data_.angle_fb_total_rad_ = 0.0f;  
    }

    // 已设置总角度0点
    int32_t delta_ecd = (int32_t)angle_raw_ecd - (int32_t)instance->fb_data_raw_last_ecd_;

    // 必须保证该函数两次调用之间ecd测出的值(这里是转子转过的角度)变化不超过半圈，否则真的转过半圈反而会被错误地认为是反向没转过半圈！
    if (delta_ecd < -(int32_t)MOTOR_DJI_M3508_ECD_HALF_RANGE) {
        // 反向转过超过半圈，认为其实是正向转过不超过半圈
        delta_ecd += (int32_t)MOTOR_DJI_M3508_ECD_RANGE;
    } else if (delta_ecd > (int32_t)MOTOR_DJI_M3508_ECD_HALF_RANGE) {
        // 正向转过超过半圈，认为其实是反向转过不超过半圈
        delta_ecd -= (int32_t)MOTOR_DJI_M3508_ECD_RANGE;
    }

    instance->total_ecd_ += delta_ecd;
    instance->fb_data_raw_last_ecd_ = angle_raw_ecd;
    instance->fb_data_.angle_fb_total_rad_ = (float)(instance->total_ecd_) * MOTOR_DJI_M3508_ECD_TO_RAD;

    return true;
}

static bool decodeFeedbackDataFromRaw(motorDJIM3508Instance_t *instance, const uint8_t *fb_data_raw, uint64_t timestamp_us)
{
    if (instance == NULL || fb_data_raw == NULL) {
        return false;
    }

    uint16_t angle_raw = (uint16_t)fb_data_raw[1] | (uint16_t)fb_data_raw[0] << 8;
    // MOTOR_DJI_M3508_FEEDBACK_ANGLE_MAX_UINT16 + 1U是因为范围是[0,8191],如果除以8191，那么映射为[0,2 * pi]，左边界和右边界重合，如果除以8192则为[0,2 * pi)
    instance->fb_data_.angle_fb_rad_ = DEG_TO_RAD * MOTOR_DJI_M3508_FEEDBACK_ANGLE_MAX_DEG * ((float)angle_raw / (float)(MOTOR_DJI_M3508_FEEDBACK_ANGLE_MAX_UINT16 + 1U));
    // 更新多圈总角度
    if (updateTotalAngle(instance, angle_raw) == false) {
        return false;
    }
    int16_t rpm_raw = (int16_t)((uint16_t)fb_data_raw[3] | ((uint16_t)fb_data_raw[2] << 8));
    instance->fb_data_.angular_velocity_fb_rads_ = RPM_TO_RADS * (float)rpm_raw;
    int16_t current_raw = (int16_t)((uint16_t)fb_data_raw[5] | (uint16_t)fb_data_raw[4] << 8);
    instance->fb_data_.current_fb_A_ = MOTOR_DJI_M3508_CTRL_CURRENT_MAX_A * ((float)current_raw / (float)MOTOR_DJI_M3508_CTRL_CURRENT_MAX_INT16);
    instance->fb_data_.temperature_c_ = (float)(fb_data_raw[6]);

    // 减速后总角度和角速度
    instance->fb_data_.angle_fb_total_reduced_rad_ = instance->fb_data_.angle_fb_total_rad_ / instance->reduction_ratio_;
    instance->fb_data_.angular_velocity_fb_reduced_rads_ = instance->fb_data_.angular_velocity_fb_rads_ / instance->reduction_ratio_;

    instance->fb_data_.timestamp_us_ = timestamp_us;

    return true;
}

motorDJIM3508Instance_t *motorDJIM3508InstanceInit(motorDJIM3508Config_t *config)
{
    if (config == NULL || config->abs_time_us_callback_ == NULL || config->reduction_ratio_ <= 0.0f) {
        return NULL;
    }

    if (m3508_memory_index_ >= MOTOR_DJI_M3508_MAX_INSTANCE_NUM) {
        return NULL;
    }

    // m3508 ID不允许重复
    if (checkMotorIDValid(config->can_instance_, config->motor_id_) == false) {
        return NULL;
    }

    motorDJIM3508Instance_t *instance = &m3508_instance_memory_[m3508_memory_index_];
    memset(instance, 0, sizeof(motorDJIM3508Instance_t));
    instance->can_instance_ = config->can_instance_;
    instance->motor_id_ = config->motor_id_;
    instance->reduction_ratio_ = config->reduction_ratio_;
    instance->tx_slot_ = ((uint8_t)config->motor_id_ - 1U) % MOTOR_DJI_M3508_MOTOR_NUM_ONE_GROUP;
    if (allocateTxGroupBaseID(config->motor_id_, &instance->tx_group_base_id_) == false) {
        return NULL;
    }
    if (allocateTxGroup(instance) == false) {
        return NULL;
    }
    instance->rx_id_ = MOTOR_DJI_M3508_RX_BASE_ID + config->motor_id_;
    instance->angle_is_initialize_ = false;
    instance->fb_data_raw_is_new_ = false;
    instance->work_status_ = MOTOR_WORK_STATUS_DISABLE; // 默认关闭，比较安全
    instance->abs_time_us_callback_ = config->abs_time_us_callback_;
    instance->name_ = config->name_;

    if (registerMotorFeedbackCallback(instance) == false) {
        return NULL;
    }

    m3508_memory_index_ ++;

    return instance;
}

bool motorDJIM3508SetCurrentRef(motorDJIM3508Instance_t *instance, float current_ref_A)
{
    if (instance == NULL) {
        return false;
    }

    float current_ref_clamped_A;
    mathClampf(current_ref_A, MOTOR_DJI_M3508_CTRL_CURRENT_MIN_A, MOTOR_DJI_M3508_CTRL_CURRENT_MAX_A, &current_ref_clamped_A);

    instance->current_ref_A_ = current_ref_clamped_A;

    return true;
}

bool motorDJIM3508SetWorkStatus(motorDJIM3508Instance_t *instance, motorWorkStatus_e work_status)
{
    if (instance == NULL) {
        return false;
    }

    instance->work_status_ = work_status;

    return true;
}

motorStatus_e motorDJIM3508GroupSendByMotorInstance(const motorDJIM3508Instance_t *anchor_instance)
{
    if (anchor_instance == NULL) {
        return MOTOR_ERROR;
    }

    int16_t tx_data[MOTOR_DJI_M3508_MOTOR_NUM_ONE_GROUP] = {0};

    motorDJIM3508Instance_t *group_head = anchor_instance->tx_group_head_;
    while (group_head != NULL) {
        if (group_head->work_status_ == MOTOR_WORK_STATUS_ENABLE) {
            convertRefFloatToInt16(group_head->current_ref_A_, &tx_data[group_head->tx_slot_]);
        } else {
            // 电流值设为0
            tx_data[group_head->tx_slot_] = 0;
        }
        group_head = group_head->tx_group_next_;
    }

    bspCANMessage_t tx_message;
    if (buildCANTxMessage(&anchor_instance->tx_group_base_id_, tx_data, &tx_message) == false) {
        return MOTOR_ERROR;
    }

    if (bspCANTransmit(anchor_instance->can_instance_, &tx_message) != BSP_CAN_OK) {
        return MOTOR_TX_FAILURE;
    }
    
    return  MOTOR_OK;
}

motorStatus_e motorDJIM3508UpdateFeedbackData(motorDJIM3508Instance_t *instance)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    // 由于中断回调中也要操作raw反馈数据，而这个函数需要解析raw数据，因此需要防止数据撕裂
    // 使用短的临界区
    uint8_t data_raw[MOTOR_DJI_M3508_CAN_RX_DLC];
    uint64_t data_timestamp_us;
    
    bspCriticalIRQState_t irq_state;
    // 进入临界区
    irq_state = bspCriticalEnter();
    if (instance->fb_data_raw_is_new_ == false) {
        // 没有新数据，直接退出临界区
        bspCriticalExit(irq_state);
        return MOTOR_NO_NEW_DATA;
    }

    // 有新数据，拷贝数据防止读写同时发生在同一块内存上
    memcpy(data_raw, instance->fb_data_raw_, MOTOR_DJI_M3508_CAN_RX_DLC);
    data_timestamp_us = instance->fb_data_raw_timestamp_us_;
    instance->fb_data_raw_is_new_ = false;

    // 数据拷贝后退出临界区
    bspCriticalExit(irq_state);

    if (decodeFeedbackDataFromRaw(instance, data_raw, data_timestamp_us) == false) {
        return MOTOR_ERROR;
    }

    return MOTOR_OK;
}

motorStatus_e motorDJIM3508GetFeedbackData(motorDJIM3508Instance_t *instance, motorFeedBackData_t *data_out)
{
    if (instance == NULL || data_out == NULL) {
        return MOTOR_ERROR;
    }

    *data_out = instance->fb_data_;

    return MOTOR_OK;
}
