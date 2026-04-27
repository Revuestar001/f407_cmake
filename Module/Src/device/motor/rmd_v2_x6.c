#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_critical.h"
#include "motor_def.h"
#include "general_math.h"
#include "angle.h"
#include "rmd_v2_x6.h"

#define MOTOR_RMD_V2_X6_CAN_RX_DLC 8U
#define MOTOR_RMD_V2_X6_RX_BASE_ID 0x240U

#define MOTOR_RMD_V2_X6_PEAK_TORQUE_NM 7.0f
#define MOTOR_RMD_V2_X6_TORQUE_CONSTANT 0.88f

#define MOTOR_RMD_V2_X6_MULTI_ROUNDS_ANGLE_PRECISION_DEG 0.01f
#define MOTOR_RMD_V2_X6_CURRENT_PRECISION_A 0.01f

#define MOTOR_RMD_V2_X6_CAN_ID_CONFIG_TX_ID 0x300U
#define MOTOR_RMD_V2_X6_CAN_ID_CONFIG_CMD 0x79U
#define MOTOR_RMD_V2_X6_CAN_ID_CONFIG_WRITE_FLAG 0U
#define MOTOR_RMD_V2_X6_CAN_ID_MIN 1U
#define MOTOR_RMD_V2_X6_CAN_ID_MAX 32U

typedef enum
{
    MOTOR_RMD_V2_X6_TX_CMD_ACTIVE_REPLY = 0xB6U,
    MOTOR_RMD_V2_X6_TX_CMD_READ_MULTI_ROUNDS_ANGLE = 0x92U,
    MOTOR_RMD_V2_X6_TX_CMD_TORQUE_LOOP = 0xA1U, // 力矩闭环控制
} motorRMDV2X6TxCommand_e;

#define MOTOR_RMD_V2_X6_ACTIVE_REPLY_ENABLE 1U
#define MOTOR_RMD_V2_X6_ACTIVE_REPLY_DISABLE 0U
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

    uint8_t fb_data_low_accuracy_raw_[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    uint8_t fb_abs_angle_high_accuracy_raw_[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    volatile bool fb_data_low_accuracy_raw_is_new_;
    volatile bool fb_abs_angle_high_accuracy_raw_is_new_;
    uint64_t fb_data_low_accuracy_raw_timestamp_us_;
    uint64_t fb_abs_angle_high_accuracy_raw_timestamp_us_;
    uint32_t fb_abs_angle_high_accuracy_timeout_us_; // 超时时间
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

    switch (rx_message->message_data_[0]) {
        case MOTOR_RMD_V2_X6_RX_CMD_TORQUE_LOOP:
            // 低精度角度、速度反馈，高精度电流反馈
            memcpy(instance->fb_data_low_accuracy_raw_, rx_message->message_data_, MOTOR_RMD_V2_X6_CAN_RX_DLC);
            instance->fb_data_low_accuracy_raw_timestamp_us_ = instance->abs_time_us_callback_();
            instance->fb_data_low_accuracy_raw_is_new_ = true;
            return;
        case MOTOR_RMD_V2_X6_RX_CMD_READ_MULTI_ROUNDS_ANGLE:
            // 高精度多圈角度反馈
            memcpy(instance->fb_abs_angle_high_accuracy_raw_, rx_message->message_data_, MOTOR_RMD_V2_X6_CAN_RX_DLC);
            instance->fb_abs_angle_high_accuracy_raw_timestamp_us_ = instance->abs_time_us_callback_();
            instance->fb_abs_angle_high_accuracy_raw_is_new_ = true;
            return;
        default:
            // 未知反馈
            break;
    }

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

static bool rawDataIsValid(uint64_t timestamp_us)
{
    return timestamp_us != 0U;
}

// 判断数据是否未过期
static bool rawDataIsFresh(uint64_t timestamp_us, uint64_t now_us, uint32_t timeout_us)
{
    return timestamp_us != 0U && now_us >= timestamp_us && (now_us - timestamp_us) <= timeout_us;
}

static float motorRMDV2X6ParseHighAccuracyAngleTotalReducedRad(const uint8_t *high_accuracy_raw)
{
    int32_t angle_abs_total_reduced_int32;

    if (high_accuracy_raw == NULL) {
        return 0.0f;
    }

    angle_abs_total_reduced_int32 = (int32_t)high_accuracy_raw[4] |
                                    (int32_t)high_accuracy_raw[5] << 8 |
                                    (int32_t)high_accuracy_raw[6] << 16 |
                                    (int32_t)high_accuracy_raw[7] << 24;

    return (float)angle_abs_total_reduced_int32 * MOTOR_RMD_V2_X6_MULTI_ROUNDS_ANGLE_PRECISION_DEG * DEG_TO_RAD;
}

static motorStatus_e motorRMDV2X6SetReadMultiRoundsAngleActiveReply(const motorRMDV2X6Instance_t *instance,
                                                                    bool enable,
                                                                    uint16_t reply_interval_10ms);

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
    instance->tx_base_id_ = MOTOR_RMD_V2_X6_SINGLE_BASE_ID;
    instance->rx_id_ = MOTOR_RMD_V2_X6_RX_BASE_ID + config->motor_id_;
    instance->fb_abs_angle_high_accuracy_timeout_us_ = config->fb_abs_angle_high_accuracy_timeout_us_;
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

#if USER_RMD_V2_X6_DIRECT_READ_92_DEBUG_ENABLE == 0U
    if (work_status == MOTOR_WORK_STATUS_ENABLE) {
        motorStatus_e motor_status = motorRMDV2X6SetReadMultiRoundsAngleActiveReply(instance, true, 1U);
        if (motor_status != MOTOR_OK) {
            return false;
        }
    }
#endif

    instance->work_status_ = work_status;

    return true;
}

static bool buildCANTxMessage(const motorRMDV2X6Instance_t *instance, const uint8_t *tx_data, bspCANMessage_t *tx_message)
{
    if (instance == NULL || tx_data == NULL || tx_message == NULL) {
        return false;
    }

    tx_message->message_header_.message_id_ = instance->tx_base_id_ + (uint32_t)instance->motor_id_;
    tx_message->message_header_.message_ide_ = 0U;
    tx_message->message_header_.message_rtr_ = 0U;
    tx_message->message_header_.message_dlc_ = MOTOR_RMD_V2_X6_CAN_RX_DLC;
    memcpy(tx_message->message_data_, tx_data, MOTOR_RMD_V2_X6_CAN_RX_DLC);

    return true;
}

static motorStatus_e motorRMDV2X6SetReadMultiRoundsAngleActiveReply(const motorRMDV2X6Instance_t *instance,
                                                                    bool enable,
                                                                    uint16_t reply_interval_10ms)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    bspCANMessage_t tx_message;
    uint8_t tx_data[MOTOR_RMD_V2_X6_CAN_RX_DLC] = {0U};
    tx_data[0] = MOTOR_RMD_V2_X6_TX_CMD_ACTIVE_REPLY;
    tx_data[1] = MOTOR_RMD_V2_X6_TX_CMD_READ_MULTI_ROUNDS_ANGLE;
    tx_data[2] = enable == true ? MOTOR_RMD_V2_X6_ACTIVE_REPLY_ENABLE : MOTOR_RMD_V2_X6_ACTIVE_REPLY_DISABLE;
    tx_data[3] = (uint8_t)(reply_interval_10ms & 0x00FFU);
    tx_data[4] = (uint8_t)((reply_interval_10ms >> 8) & 0x00FFU);
    if (buildCANTxMessage(instance, tx_data, &tx_message) == false) {
        return MOTOR_ERROR;
    }

    if (bspCANTransmit(instance->can_instance_, &tx_message) != BSP_CAN_OK) {
        return MOTOR_TX_FAILURE;
    }

    return MOTOR_OK;
}

static bool decodeFeedbackDataFromRaw(motorRMDV2X6Instance_t *instance,
                                      const uint8_t *low_accuracy,
                                      uint64_t low_accuracy_timestamp_us,
                                      const uint8_t *high_accuracy,
                                      bool use_high_accuracy_angle)
{
    if (instance == NULL || low_accuracy == NULL) {
        return false;
    }

    int16_t angle_reduced_raw_int16 = (int16_t)low_accuracy[6] | (int16_t)low_accuracy[7] << 8;
    instance->fb_data_.angle_fb_total_reduced_rad_ = (float)angle_reduced_raw_int16 * DEG_TO_RAD;
    if (use_high_accuracy_angle == true && high_accuracy != NULL) {
        instance->fb_data_.angle_fb_total_reduced_rad_ =
            motorRMDV2X6ParseHighAccuracyAngleTotalReducedRad(high_accuracy);
    }
    instance->fb_data_.angle_fb_total_rad_ = instance->reduction_ratio_ * instance->fb_data_.angle_fb_total_reduced_rad_;
    instance->fb_data_.angle_fb_rad_ = mathAngleWrapPi(instance->fb_data_.angle_fb_total_rad_);

    int16_t dps_reduced_raw_int16 = (int16_t)low_accuracy[4] | (int16_t)low_accuracy[5] << 8;
    // 反馈的就是已经减速后的数据！
    instance->fb_data_.angular_velocity_fb_reduced_rads_ = (float)dps_reduced_raw_int16 * DEG_TO_RAD;
    instance->fb_data_.angular_velocity_fb_rads_ = instance->fb_data_.angular_velocity_fb_reduced_rads_ * instance->reduction_ratio_;

    int16_t current_raw = (int16_t)low_accuracy[2] | (int16_t)low_accuracy[3] << 8;
    instance->fb_data_.current_fb_A_ = (float)current_raw * MOTOR_RMD_V2_X6_CURRENT_PRECISION_A;

    instance->fb_data_.temperature_c_ = (float)((int8_t)low_accuracy[1]);

    instance->fb_data_.timestamp_us_ = low_accuracy_timestamp_us;

    return true;
}

// 只发送力矩闭环控制报文
motorStatus_e motorRMDV2X6SendEffortCommand(const motorRMDV2X6Instance_t *instance)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    int16_t torque_ref = (int16_t)(100.0f * instance->effort_ref_Nm_ / MOTOR_RMD_V2_X6_TORQUE_CONSTANT);

    bspCANMessage_t torque_message;
    uint8_t torque_data[MOTOR_RMD_V2_X6_CAN_RX_DLC] = {0U};
    torque_data[0] = MOTOR_RMD_V2_X6_TX_CMD_TORQUE_LOOP;
    torque_data[4] = (uint8_t)torque_ref; // 低字节在前
    torque_data[5] = (uint8_t)(torque_ref >> 8);
    if (buildCANTxMessage(instance, torque_data, &torque_message) == false) {
        return MOTOR_ERROR;
    }

    if (bspCANTransmit(instance->can_instance_, &torque_message) != BSP_CAN_OK) {
        return MOTOR_TX_FAILURE;
    }
    
    return  MOTOR_OK;
}

// 只发送请求读取高精度多圈角度报文
motorStatus_e motorRMDV2X6SendReadMultiRoundsAngleCommand(const motorRMDV2X6Instance_t *instance)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    bspCANMessage_t tx_message;
    uint8_t tx_data[MOTOR_RMD_V2_X6_CAN_RX_DLC] = {0U};
    tx_data[0] = MOTOR_RMD_V2_X6_TX_CMD_READ_MULTI_ROUNDS_ANGLE;
    if (buildCANTxMessage(instance, tx_data, &tx_message) == false) {
        return MOTOR_ERROR;
    }

    if (bspCANTransmit(instance->can_instance_, &tx_message) != BSP_CAN_OK) {
        return MOTOR_TX_FAILURE;
    }
    
    return MOTOR_OK;
}

motorStatus_e motorRMDV2X6UpdateFeedbackData(motorRMDV2X6Instance_t *instance)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    uint8_t data_low_accuracy_raw[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    uint8_t data_high_accuracy_raw[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    uint64_t data_low_accuracy_raw_timestamp_us;
    uint64_t data_high_accuracy_raw_timestamp_us;
    bool data_low_accuracy_raw_is_new;
    bool data_high_accuracy_raw_is_new;
    bool feedback_is_updated = false;

    bspCriticalIRQState_t irq_state;
    irq_state = bspCriticalEnter();

    data_low_accuracy_raw_is_new = instance->fb_data_low_accuracy_raw_is_new_;
    data_high_accuracy_raw_is_new = instance->fb_abs_angle_high_accuracy_raw_is_new_;
    memcpy(data_low_accuracy_raw, instance->fb_data_low_accuracy_raw_, MOTOR_RMD_V2_X6_CAN_RX_DLC);
    memcpy(data_high_accuracy_raw, instance->fb_abs_angle_high_accuracy_raw_, MOTOR_RMD_V2_X6_CAN_RX_DLC);
    data_low_accuracy_raw_timestamp_us = instance->fb_data_low_accuracy_raw_timestamp_us_;
    data_high_accuracy_raw_timestamp_us = instance->fb_abs_angle_high_accuracy_raw_timestamp_us_;
    instance->fb_data_low_accuracy_raw_is_new_ = false;
    instance->fb_abs_angle_high_accuracy_raw_is_new_ = false;

    bspCriticalExit(irq_state);

    if ((data_low_accuracy_raw_is_new == true || data_high_accuracy_raw_is_new == true) &&
        rawDataIsValid(data_low_accuracy_raw_timestamp_us) == true) {
        bool use_high_accuracy_angle = false;
        if (instance->abs_time_us_callback_ != NULL &&
            rawDataIsFresh(data_high_accuracy_raw_timestamp_us,
                           instance->abs_time_us_callback_(),
                           instance->fb_abs_angle_high_accuracy_timeout_us_) == true) {
            use_high_accuracy_angle = true;
        }
        if (decodeFeedbackDataFromRaw(instance,
                                      data_low_accuracy_raw,
                                      data_low_accuracy_raw_timestamp_us,
                                      data_high_accuracy_raw,
                                      use_high_accuracy_angle) == false) {
            return MOTOR_ERROR;
        }
        feedback_is_updated = true;
    }

    return feedback_is_updated == true ? MOTOR_OK : MOTOR_NO_NEW_DATA;
}

motorStatus_e motorRMDV2X6GetFeedbackData(motorRMDV2X6Instance_t *instance, motorFeedBackData_t *data_out)
{
    if (instance == NULL || data_out == NULL) {
        return MOTOR_ERROR;
    }

    if (instance->fb_data_.timestamp_us_ == 0U) {
        return MOTOR_NO_NEW_DATA;
    }

    if (instance->abs_time_us_callback_ == NULL) {
        return MOTOR_ERROR;
    }

    uint64_t now_us = instance->abs_time_us_callback_();
    // 主闭环仍以 A1 状态新鲜度为基础；高精度角若掉线则自动退回低精度角。
    if (rawDataIsFresh(instance->fb_data_.timestamp_us_,
                       now_us,
                       instance->fb_abs_angle_high_accuracy_timeout_us_) == false) {
        return MOTOR_ERROR;
    }

    *data_out = instance->fb_data_;

    return MOTOR_OK;
}

motorStatus_e motorRMDV2X6GetHighAccuracyAngleDebugData(motorRMDV2X6Instance_t *instance,
                                                        motorRMDV2X6HighAccuracyAngleDebugData_t *data_out)
{
    uint8_t high_accuracy_raw[MOTOR_RMD_V2_X6_CAN_RX_DLC];
    uint64_t high_accuracy_timestamp_us;
    bspCriticalIRQState_t irq_state;

    if (instance == NULL || data_out == NULL) {
        return MOTOR_ERROR;
    }

    irq_state = bspCriticalEnter();
    memcpy(high_accuracy_raw, instance->fb_abs_angle_high_accuracy_raw_, MOTOR_RMD_V2_X6_CAN_RX_DLC);
    high_accuracy_timestamp_us = instance->fb_abs_angle_high_accuracy_raw_timestamp_us_;
    bspCriticalExit(irq_state);

    if (rawDataIsValid(high_accuracy_timestamp_us) == false) {
        return MOTOR_NO_NEW_DATA;
    }

    data_out->timestamp_us_ = high_accuracy_timestamp_us;
    data_out->angle_fb_total_reduced_rad_ = motorRMDV2X6ParseHighAccuracyAngleTotalReducedRad(high_accuracy_raw);
    data_out->angle_fb_total_rad_ = data_out->angle_fb_total_reduced_rad_ * instance->reduction_ratio_;

    return MOTOR_OK;
}

motorStatus_e motorRMDV2X6SetHighAccuracyAngleActiveReplyDebug(motorRMDV2X6Instance_t *instance,
                                                               bool enable,
                                                               uint16_t reply_interval_10ms)
{
    return motorRMDV2X6SetReadMultiRoundsAngleActiveReply(instance, enable, reply_interval_10ms);
}

#if MOTOR_RMD_V2_X6_ENABLE_CAN_ID_CONFIG
motorStatus_e motorRMDV2X6SetSingleMotorCANID(bspCANInstance_t *can_instance, uint16_t can_id)
{
    if (can_instance == NULL) {
        return MOTOR_ERROR;
    }

    if (can_id < MOTOR_RMD_V2_X6_CAN_ID_MIN || can_id > MOTOR_RMD_V2_X6_CAN_ID_MAX) {
        return MOTOR_ERROR;
    }

    bspCANMessage_t tx_message = {0};
    tx_message.message_header_.message_id_ = MOTOR_RMD_V2_X6_CAN_ID_CONFIG_TX_ID;
    tx_message.message_header_.message_ide_ = 0U;
    tx_message.message_header_.message_rtr_ = 0U;
    tx_message.message_header_.message_dlc_ = MOTOR_RMD_V2_X6_CAN_RX_DLC;
    tx_message.message_data_[0] = MOTOR_RMD_V2_X6_CAN_ID_CONFIG_CMD;
    tx_message.message_data_[2] = MOTOR_RMD_V2_X6_CAN_ID_CONFIG_WRITE_FLAG;
    tx_message.message_data_[7] = (uint8_t)can_id;

    if (bspCANTransmit(can_instance, &tx_message) != BSP_CAN_OK) {
        return MOTOR_TX_FAILURE;
    }

    return MOTOR_OK;
}
#endif

