#include "motor_def.h"
#include "projdefs.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_board.h"
#include "cmsis_os2.h"
#include "dji_m3508.h"
#include "motor_actuator.h"
#include "motor_actuator_group.h"
#include "pid.h"
#include "general_math.h"
#include "motor_actuator_dji_adapter.h"
#include "app_remote_control.h"
#include "app_chassis.h"
#include "app_def.h"
#include "user_def.h"

typedef struct app_chassis
{
    moduleMotorActuator_t actuator_[2U];
    moduleMotorActuatorGroup_t actuator_group_;

    uint8_t error_count_;
} appChassis_t;

static appChassis_t app_chassis_ = {0};

#define APP_CHASSIS_RC_STICK_MAX_ABS 1000.0f
#define APP_CHASSIS_MAX_LINEAR_SPEED_REF_RADS 3.0f
#define APP_CHASSIS_MAX_TURN_SPEED_REF_RADS 2.0f

static float appChassisMapRCStickToSpeedRef(int16_t stick_value, float max_abs_speed_ref_rads)
{
    return ((float)stick_value / APP_CHASSIS_RC_STICK_MAX_ABS) * max_abs_speed_ref_rads;
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

    mathClampf(motor_left_speed_ref_rads,
               -motor_speed_ref_limit_rads,
               motor_speed_ref_limit_rads,
               &left_ref->velocity_ref_rads_);
    mathClampf(motor_right_speed_ref_rads,
               -motor_speed_ref_limit_rads,
               motor_speed_ref_limit_rads,
               &right_ref->velocity_ref_rads_);
}

static bool appChassisInit(void)
{
    motorDJIM3508Config_t m3508_config = {
        .can_instance_ = bspBoardGetCANInstance(BSP_CAN_1),
        .motor_id_ = MOTOR_DJI_M3508_MOTOR_ID_1,
        .reduction_ratio_ = 19.0f,
        .abs_time_us_callback_ = bspDWTGetAbsTimeUs,
        .name_ = "m3508_1",
    };
    
    motorDJIM3508Instance_t *motor1 = motorDJIM3508InstanceInit(&m3508_config);
    if (motor1 == NULL) {
        return false;
    }

    m3508_config.motor_id_ = MOTOR_DJI_M3508_MOTOR_ID_2;
    m3508_config.name_ = "m3508_2";
    motorDJIM3508Instance_t *motor2 = motorDJIM3508InstanceInit(&m3508_config);
    if (motor2 == NULL) {
        return false;
    }

    algorithmPIDConfig_t angle_pid_config = {
        .k_P_ = 4.0f,
        .k_I_ = 0.0f,
        .k_D_ = 0.0f,
        .integral_lower_bound_ = -0.1f,
        .integral_upper_bound_ = 0.1f,
        .output_lower_bound_ = -4.0f,
        .output_upper_bound_ = 4.0f,
        .integral_method_ = ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID,
        .derivative_method_ = ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE,
        .derivative_filter_ = NULL,
        .get_time_us_callback_ = bspDWTGetAbsTimeUs,
    };
    algorithmPIDConfig_t velocity_pid_config = {
        .k_P_ = 3.0f,
        .k_I_ = 0.5f,
        .k_D_ = 0.0f,
        .integral_lower_bound_ = -2.0f,
        .integral_upper_bound_ = 2.0f,
        .output_lower_bound_ = -5.0f,
        .output_upper_bound_ = 5.0f,
        .integral_method_ = ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID,
        .derivative_method_ = ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE,
        .derivative_filter_ = NULL,
        .get_time_us_callback_ = bspDWTGetAbsTimeUs,
    };

    moduleMotorActuatorConfig_t actuator_config = {
        .motor_instance_ = motor1,
        .feedback_source_ = motor1,
        .angle_pid_config_ = angle_pid_config,
        .angular_velocity_pid_config_ = velocity_pid_config,
        .command_ops_ = adapterGetDJIM3508CommandOps(),
        .feedback_ops_ = adapterGetDJIM3508FeedbackOps(),
        .control_loop_ = MODULE_MOTOR_ACTUATOR_CTRL_LOOP_SPEED,
        .command_type_ = MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT,
        .feedback_side_ = MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_OUTPUT,
        .command_sign_ = 1.0f,
        .feedback_sign_ = 1.0f,
    };

    if (moduleMotorActuatorInit(&app_chassis_.actuator_[0], &actuator_config) == false) {
        return false;
    }

    actuator_config.motor_instance_ = motor2;
    actuator_config.feedback_source_ = motor2;
    if (moduleMotorActuatorInit(&app_chassis_.actuator_[1], &actuator_config) == false) {
        return false;
    }
    
    moduleMotorActuator_t *actuator_temp[4U] = {0};
    actuator_temp[0] = &app_chassis_.actuator_[0];
    actuator_temp[1] = &app_chassis_.actuator_[1];
    if (moduleMotorActuatorGroupInit(&app_chassis_.actuator_group_, actuator_temp, adapterGetDJIM3508GroupCommandOps()) == false) {
        return false;
    }
    
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
}

void appChassisTaskEntry(void *argument)
{
    (void)argument;

    while (appChassisInit() != true) {
        osDelay(pdMS_TO_TICKS(100));
    }
    while (moduleMotorActuatorEnableMotor(&app_chassis_.actuator_[0]) != true) {
        osDelay(pdMS_TO_TICKS(100));
    }
    while (moduleMotorActuatorEnableMotor(&app_chassis_.actuator_[1]) != true) {
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
        appRemoteControlCommand_t command_new;
        if (appRemoteControlReceiveCommand(&command_new, 0U) == true) {
            remote_control_command = command_new;
        }

        appChassisBuildVelocityCommandFromRemoteControl(&remote_control_command, &left_ref, &right_ref);

        motor_status = moduleMotorActuatorUpdateCommand(&app_chassis_.actuator_[0], &left_ref);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }
        motor_status = moduleMotorActuatorUpdateCommand(&app_chassis_.actuator_[1], &right_ref);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }

        motor_status = moduleMotorActuatorGroupCommit(&app_chassis_.actuator_group_);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }

        osDelay(pdMS_TO_TICKS(1));
    }
}


