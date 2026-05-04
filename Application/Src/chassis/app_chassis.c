#include "cmsis_os2.h"
#include "projdefs.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_board.h"

#include "motor_def.h"
#include "rmd_v2_x6.h"
#include "app_remote_control.h"
#include "app_chassis.h"
#include "chassis_platform.h"
#include "app_def.h"
#include "user_def.h"
#include "app_topics.h"
#include "topic_bus.h"
#include "msg_rc_command.h"

typedef struct app_chassis
{
    appChassisPlatform_t platform_;
    moduleMotorActuator_t *hip_actuator_;

    motorRMDV2X6Instance_t *x6_debug_motor_;

    motorRMDV2X6HighAccuracyAngleDebugData_t x6_debug_latest_angle_;
    motorStatus_e x6_debug_last_request_status_;
    motorStatus_e x6_debug_last_angle_status_;
    motorStatus_e x6_debug_last_active_reply_config_status_;
    uint32_t x6_debug_request_count_;
    uint32_t x6_debug_reply_count_;
    uint32_t x6_debug_active_reply_config_count_;
    bool x6_debug_active_reply_config_sent_;

    moduleTopicSubscription_t rc_sub_handle_;

    uint8_t error_count_;
} appChassis_t;

static appChassis_t app_chassis_ = {0};

#define APP_CHASSIS_RC_STICK_MAX_ABS 1000.0f
#define APP_CHASSIS_MAX_LINEAR_SPEED_REF_RADS 3.0f
#define APP_CHASSIS_MAX_TURN_SPEED_REF_RADS 2.0f
#define APP_CHASSIS_HIP_ACTUATOR_NAME "x6_1"

static void appChassisDebugUpdateX692Snapshot(void)
{
    motorRMDV2X6HighAccuracyAngleDebugData_t angle_debug_data;
    motorRMDV2X6Instance_t *x6 = app_chassis_.x6_debug_motor_;

    if (x6 == NULL) {
        return;
    }

    app_chassis_.x6_debug_last_angle_status_ = motorRMDV2X6GetHighAccuracyAngleDebugData(x6, &angle_debug_data);
    if (app_chassis_.x6_debug_last_angle_status_ == MOTOR_OK &&
        angle_debug_data.timestamp_us_ != app_chassis_.x6_debug_latest_angle_.timestamp_us_) {
        app_chassis_.x6_debug_latest_angle_ = angle_debug_data;
        app_chassis_.x6_debug_reply_count_++;
    }
}

static void appChassisDebugDirectReadX692(void)
{
#if USER_RMD_V2_X6_DIRECT_READ_92_DEBUG_ENABLE
    static uint32_t last_request_tick_ms = 0U;
    uint32_t now_tick_ms = osKernelGetTickCount();
    motorRMDV2X6Instance_t *x6 = app_chassis_.x6_debug_motor_;

    if (x6 == NULL) {
        return;
    }

    appChassisDebugUpdateX692Snapshot();

    if ((now_tick_ms - last_request_tick_ms) < USER_RMD_V2_X6_DIRECT_READ_92_DEBUG_PERIOD_MS) {
        return;
    }
    last_request_tick_ms = now_tick_ms;

    app_chassis_.x6_debug_last_request_status_ = motorRMDV2X6SendReadMultiRoundsAngleCommand(x6);
    if (app_chassis_.x6_debug_last_request_status_ != MOTOR_OK) {
        app_chassis_.error_count_ ++;
        return;
    }
    app_chassis_.x6_debug_request_count_++;
#endif
}

static void appChassisDebugActiveReplyX692(void)
{
#if USER_RMD_V2_X6_ACTIVE_REPLY_92_DEBUG_ENABLE
    static uint32_t last_config_tick_ms = 0U;
    motorRMDV2X6Instance_t *x6 = app_chassis_.x6_debug_motor_;
    uint32_t now_tick_ms = osKernelGetTickCount();

    if (x6 == NULL) {
        return;
    }

    appChassisDebugUpdateX692Snapshot();

    if (app_chassis_.x6_debug_active_reply_config_sent_ == true) {
        return;
    }

    if (app_chassis_.x6_debug_active_reply_config_count_ > 0U &&
        (now_tick_ms - last_config_tick_ms) < USER_RMD_V2_X6_ACTIVE_REPLY_92_DEBUG_CONFIG_RETRY_PERIOD_MS) {
        return;
    }
    last_config_tick_ms = now_tick_ms;
    app_chassis_.x6_debug_active_reply_config_count_++;

    app_chassis_.x6_debug_last_active_reply_config_status_ =
        motorRMDV2X6SetHighAccuracyAngleActiveReplyDebug(x6,
                                                         true,
                                                         USER_RMD_V2_X6_ACTIVE_REPLY_92_DEBUG_REPLY_PERIOD_10MS);
    if (app_chassis_.x6_debug_last_active_reply_config_status_ != MOTOR_OK) {
        app_chassis_.error_count_++;
        return;
    }

    app_chassis_.x6_debug_active_reply_config_sent_ = true;
#endif
}

static float appChassisMapRCStickToSpeedRef(int16_t stick_value, float max_abs_speed_ref_rads)
{
    return ((float)stick_value / APP_CHASSIS_RC_STICK_MAX_ABS) * max_abs_speed_ref_rads;
}

static bool appChassisBindPlatform(appChassis_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    instance->hip_actuator_ = appChassisPlatformFindActuatorByName(&instance->platform_,
                                                                   APP_CHASSIS_HIP_ACTUATOR_NAME);
    if (instance->hip_actuator_ == NULL) {
        return false;
    }

    return true;
}

static void appChassisBuildVelocityCommandFromRemoteControl(const appRemoteControlCommand_t *command,
                                                            moduleMotorActuatorCommandRef_t *left_ref,
                                                            moduleMotorActuatorCommandRef_t *right_ref)
{
    if (left_ref == NULL || right_ref == NULL) {
        return;
    }

    memset(left_ref, 0, sizeof(*left_ref));
    memset(right_ref, 0, sizeof(*right_ref));

    if (command == NULL ||
        command->state_ != APP_REMOTE_CONTROL_STATE_CONTROL ||
        command->armed_ == false) {
        return;
    }

    float linear_speed_ref_rads = appChassisMapRCStickToSpeedRef(command->linear_speed_,
                                                                 APP_CHASSIS_MAX_LINEAR_SPEED_REF_RADS);
    float turn_speed_ref_rads = appChassisMapRCStickToSpeedRef(command->angular_speed_,
                                                               APP_CHASSIS_MAX_TURN_SPEED_REF_RADS);
    float motor_left_speed_ref_rads = linear_speed_ref_rads - turn_speed_ref_rads;
    float motor_right_speed_ref_rads = -(linear_speed_ref_rads + turn_speed_ref_rads);
    float motor_speed_ref_limit_rads = APP_CHASSIS_MAX_LINEAR_SPEED_REF_RADS + APP_CHASSIS_MAX_TURN_SPEED_REF_RADS;

    // mathClampf(motor_left_speed_ref_rads,
    //            -motor_speed_ref_limit_rads,
    //            motor_speed_ref_limit_rads,
    //            &left_ref->velocity_ref_rads_);
    // mathClampf(motor_right_speed_ref_rads,
    //            -motor_speed_ref_limit_rads,
    //            motor_speed_ref_limit_rads,
    //            &right_ref->velocity_ref_rads_);
    mathClampf(motor_left_speed_ref_rads,
               -motor_speed_ref_limit_rads,
               motor_speed_ref_limit_rads,
               &left_ref->position_ref_rad_);
    mathClampf(motor_right_speed_ref_rads,
               -motor_speed_ref_limit_rads,
               motor_speed_ref_limit_rads,
               &right_ref->position_ref_rad_);
}

static bool appChassisInit(void)
{
    if (moduleTopicBusSubscribe(appTopicsGet(APP_TOPICS_RC_COMMAND), &app_chassis_.rc_sub_handle_) == false) {
        return false;
    }

#if USER_RMD_V2_X6_DIRECT_READ_92_DEBUG_ENABLE || USER_RMD_V2_X6_ACTIVE_REPLY_92_DEBUG_ENABLE
    motorRMDV2X6Config_t x6_config = {
        .motor_id_ = MOTOR_RMD_V2_X6_MOTOR_ID_2,
        .can_instance_ = bspBoardGetCANInstance(BSP_CAN_1),
        .fb_abs_angle_high_accuracy_timeout_us_ = 20000U,
        .reduction_ratio_ = 6.0f,
        .abs_time_us_callback_ = bspDWTGetAbsTimeUs,
        .name_ = "x6_1",
    };

    motorRMDV2X6Instance_t *x6 = motorRMDV2X6InstanceInit(&x6_config);
    if (x6 == NULL) {
        return false;
    }
    app_chassis_.x6_debug_motor_ = x6;

    bspCANStatus_e can_status;
    can_status = bspCANSetFilter(bspBoardGetCANInstance(BSP_CAN_1));
    if (can_status != BSP_CAN_OK) {
        return false;
    }
    can_status = bspCANStart(bspBoardGetCANInstance(BSP_CAN_1));
    if (can_status != BSP_CAN_OK) {
        return false;
    }

    return true;
#else
    if (appChassisPlatformInit(&app_chassis_.platform_) == false) {
        return false;
    }

    return appChassisBindPlatform(&app_chassis_);
#endif
}

void appChassisTaskEntry(void *argument)
{
    (void)argument;

    while (appChassisInit() != true) {
        osDelay(pdMS_TO_TICKS(100));
    }
#if USER_RMD_V2_X6_DIRECT_READ_92_DEBUG_ENABLE
    for (;;) {
        appChassisDebugDirectReadX692();
        osDelay(pdMS_TO_TICKS(1));
    }
#elif USER_RMD_V2_X6_ACTIVE_REPLY_92_DEBUG_ENABLE
    for (;;) {
        appChassisDebugActiveReplyX692();
        osDelay(pdMS_TO_TICKS(1));
    }
#else
    while (moduleMotorActuatorEnableMotor(app_chassis_.hip_actuator_) != true) {
        osDelay(pdMS_TO_TICKS(100));
    }
    moduleMotorActuatorCommandRef_t left_ref = {0};
    moduleMotorActuatorCommandRef_t right_ref = {0};
    appRemoteControlCommand_t remote_control_command = {
        .state_ = APP_REMOTE_CONTROL_STATE_LOST,
        .armed_ = false,
        .drive_mode_ = APP_REMOTE_CONTROL_DRIVE_MODE_MANUAL,
        .linear_speed_ = 0,
        .angular_speed_ = 0,
    };

    motorStatus_e motor_status;

    for (;;) {
        msgRCCommand_t rc_msg;
        if (moduleTopicBusCopy(&app_chassis_.rc_sub_handle_, &rc_msg) == true) {
            remote_control_command.state_ = rc_msg.state_;
            remote_control_command.armed_ = rc_msg.armed_;
            remote_control_command.drive_mode_ = rc_msg.drive_mode_;
            remote_control_command.linear_speed_ = rc_msg.linear_speed_;
            remote_control_command.angular_speed_ = rc_msg.angular_speed_;
        }
        
        appChassisBuildVelocityCommandFromRemoteControl(&remote_control_command, &left_ref, &right_ref);

        motor_status = moduleMotorActuatorUpdateCommand(app_chassis_.hip_actuator_, &left_ref);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }

        motor_status = moduleMotorActuatorCommit(app_chassis_.hip_actuator_);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }

        appChassisDebugDirectReadX692();

        osDelay(pdMS_TO_TICKS(1));
    }
#endif
}
