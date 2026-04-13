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
#include "motor_actuator_dji_adapter.h"
#include "app_chassis.h"
#include "app_def.h"
#include "user_def.h"

typedef struct app_chassis
{
    moduleMotorActuator_t actuator_;
    moduleMotorActuatorGroup_t actuator_group_;

    uint8_t error_count_;
} appChassis_t;

static appChassis_t app_chassis_ = {0};

static bool appChassisInit(void)
{
    motorDJIM3508Config_t m3508_config = {
        .can_instance_ = bspBoardGetCANInstance(BSP_CAN_1),
        .motor_id_ = MOTOR_DJI_M3508_MOTOR_ID_1,
        .reduction_ratio_ = 19.0f,
        .abs_time_us_callback_ = bspDWTGetAbsTimeUs,
        .name_ = "m3508_1",
    };
    motorDJIM3508Instance_t *motor = motorDJIM3508InstanceInit(&m3508_config);
    if (motor == NULL) {
        return false;
    }

    algorithmPIDConfig_t angle_pid_config = {
        .k_P_ = 2.0f,
        .k_I_ = 0.0f,
        .k_D_ = 0.0f,
        .integral_lower_bound_ = -0.1f,
        .integral_upper_bound_ = 0.1f,
        .output_lower_bound_ = -2.0f,
        .output_upper_bound_ = 2.0f,
        .integral_method_ = ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID,
        .derivative_method_ = ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE,
        .derivative_filter_ = NULL,
        .get_time_us_callback_ = bspDWTGetAbsTimeUs,
    };
    algorithmPIDConfig_t velocity_pid_config = {
        .k_P_ = 1.0f,
        .k_I_ = 0.5f,
        .k_D_ = 0.0f,
        .integral_lower_bound_ = -1.0f,
        .integral_upper_bound_ = 1.0f,
        .output_lower_bound_ = -2.0f,
        .output_upper_bound_ = 2.0f,
        .integral_method_ = ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID,
        .derivative_method_ = ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE,
        .derivative_filter_ = NULL,
        .get_time_us_callback_ = bspDWTGetAbsTimeUs,
    };

    moduleMotorActuatorConfig_t actuator_config = {
        .motor_instance_ = motor,
        .feedback_source_ = motor,
        .angle_pid_config_ = angle_pid_config,
        .angular_velocity_pid_config_ = velocity_pid_config,
        .command_ops_ = adapterGetDJIM3508CommandOps(),
        .feedback_ops_ = adapterGetDJIM3508FeedbackOps(),
        .control_loop_ = MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE_AND_SPEED,
        .command_type_ = MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT,
        .feedback_side_ = MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_OUTPUT,
        .command_sign_ = 1.0f,
        .feedback_sign_ = 1.0f,
    };

    if (moduleMotorActuatorInit(&app_chassis_.actuator_, &actuator_config) == false) {
        return false;
    }
    moduleMotorActuator_t *actuator_temp[4U] = {0};
    actuator_temp[0] = &app_chassis_.actuator_;
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
    while (moduleMotorActuatorEnableMotor(&app_chassis_.actuator_) != true) {
        osDelay(pdMS_TO_TICKS(100));
    }
    moduleMotorActuatorCommandRef_t ref = {
        .effort_ref_ = 0.0f,
        .velocity_ref_rads_ = 0.0f,
        .position_ref_rad_ = 1.0f,
        .mit_kp_ = 0.0f,
        .mit_kd_ = 0.0f,
    };

    motorStatus_e motor_status;

    for (;;) {
        motor_status = moduleMotorActuatorUpdateCommand(&app_chassis_.actuator_, &ref);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }
        motor_status = moduleMotorActuatorGroupCommit(&app_chassis_.actuator_group_);
        if (motor_status != MOTOR_OK) {
            app_chassis_.error_count_ ++;
        }
        // ref.position_ref_rad_ += 0.01f * DEG_TO_RAD;

        osDelay(pdMS_TO_TICKS(1));
    }
}