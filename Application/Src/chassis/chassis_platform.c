#include <stdbool.h>
#include <string.h>

#include "chassis_platform.h"

#include "bsp_board.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "dji_m3508.h"
#include "motor_actuator_dji_adapter.h"
#include "motor_actuator_myactuator_adapter.h"
#include "pid.h"
#include "rmd_v2_x6.h"

#define APP_CHASSIS_PLATFORM_ACTUATOR_STORAGE_CAPACITY \
    (MOTOR_RMD_V2_X6_MAX_INSTANCE_NUM + MOTOR_DJI_M3508_MAX_INSTANCE_NUM)

#define APP_CHASSIS_PLATFORM_RMD_V2_X6_SPEC_COUNT 1U
#define APP_CHASSIS_PLATFORM_DJI_M3508_SPEC_COUNT 0U

typedef struct app_chassis_platform_actuator_entry
{
    const char *name_;
    moduleMotorActuator_t *actuator_;
} appChassisPlatformActuatorEntry_t;

typedef struct app_chassis_platform_actuator_tuning
{
    algorithmPIDConfig_t angle_pid_config_;
    algorithmPIDConfig_t velocity_pid_config_;
    moduleMotorActuatorControlLoop_e control_loop_;
    moduleMotorActuatorCommandType_e command_type_;
    moduleMotorActuatorFeedbackSide_e feedback_side_;
    float command_sign_;
    float feedback_sign_;
} appChassisPlatformActuatorTuning_t;

typedef struct app_chassis_platform_rmd_v2_x6_spec
{
    const char *name_;
    bspCANId_e can_id_;
    motorRMDV2X6MotorID_e motor_id_;
    float reduction_ratio_;
    uint32_t fb_abs_angle_high_accuracy_timeout_us_;
    appChassisPlatformActuatorTuning_t tuning_;
} appChassisPlatformRMDV2X6Spec_t;

typedef struct app_chassis_platform_dji_m3508_spec
{
    const char *name_;
    bspCANId_e can_id_;
    motorDJIM3508MotorID_e motor_id_;
    float reduction_ratio_;
    appChassisPlatformActuatorTuning_t tuning_;
} appChassisPlatformDJIM3508Spec_t;

struct app_chassis_platform_context
{
    moduleMotorActuator_t actuator_storage_[APP_CHASSIS_PLATFORM_ACTUATOR_STORAGE_CAPACITY];
    appChassisPlatformActuatorEntry_t actuator_entry_[APP_CHASSIS_PLATFORM_ACTUATOR_STORAGE_CAPACITY];
    uint32_t actuator_count_;
};

static const appChassisPlatformRMDV2X6Spec_t app_chassis_platform_rmd_v2_x6_spec_list[APP_CHASSIS_PLATFORM_RMD_V2_X6_SPEC_COUNT] = {
    {
        .name_ = "x6_1",
        .can_id_ = BSP_CAN_1,
        .motor_id_ = MOTOR_RMD_V2_X6_MOTOR_ID_2,
        .reduction_ratio_ = 6.0f,
        .fb_abs_angle_high_accuracy_timeout_us_ = 20000U,
        .tuning_ = {
            .angle_pid_config_ =
                {
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
                },
            .velocity_pid_config_ =
                {
                    .k_P_ = 3.0f,
                    .k_I_ = 0.5f,
                    .k_D_ = 0.0f,
                    .integral_lower_bound_ = -2.0f,
                    .integral_upper_bound_ = 2.0f,
                    .output_lower_bound_ = -2.0f,
                    .output_upper_bound_ = 2.0f,
                    .integral_method_ = ALGORITHM_PID_INTEGRAL_METHOD_TRAPEZOID,
                    .derivative_method_ = ALGORITHM_PID_DIFFERENTIAL_METHOD_ESTIMATE,
                    .derivative_filter_ = NULL,
                    .get_time_us_callback_ = bspDWTGetAbsTimeUs,
                },
            .control_loop_ = MODULE_MOTOR_ACTUATOR_CTRL_LOOP_ANGLE_AND_SPEED,
            .command_type_ = MODULE_MOTOR_ACTUATOR_CMD_TYPE_EFFORT,
            .feedback_side_ = MODULE_MOTOR_ACTUATOR_FEEDBACK_SIDE_OUTPUT,
            .command_sign_ = 1.0f,
            .feedback_sign_ = 1.0f,
        },
    },
};

static appChassisPlatformContext_t app_chassis_platform_context_ = {0};

static bool appChassisPlatformAllocateActuatorSlot(appChassisPlatformContext_t *context,
                                                   moduleMotorActuator_t **actuator_out)
{
    if (context == NULL || actuator_out == NULL) {
        return false;
    }

    if (context->actuator_count_ >= APP_CHASSIS_PLATFORM_ACTUATOR_STORAGE_CAPACITY) {
        return false;
    }

    *actuator_out = &context->actuator_storage_[context->actuator_count_];
    return true;
}

static bool appChassisPlatformAppendActuatorEntry(appChassisPlatformContext_t *context,
                                                  const char *name,
                                                  moduleMotorActuator_t *actuator)
{
    uint32_t index;

    if (context == NULL || name == NULL || actuator == NULL) {
        return false;
    }

    if (context->actuator_count_ >= APP_CHASSIS_PLATFORM_ACTUATOR_STORAGE_CAPACITY) {
        return false;
    }

    index = context->actuator_count_;
    context->actuator_entry_[index].name_ = name;
    context->actuator_entry_[index].actuator_ = actuator;
    context->actuator_count_++;

    return true;
}

static bool appChassisPlatformBuildActuator(moduleMotorActuator_t *actuator_out,
                                            void *motor_instance,
                                            void *feedback_source,
                                            const moduleMotorActuatorCommandOps_t *command_ops,
                                            const moduleMotorActuatorFeedbackOps_t *feedback_ops,
                                            const appChassisPlatformActuatorTuning_t *tuning)
{
    moduleMotorActuatorConfig_t actuator_config;

    if (actuator_out == NULL || motor_instance == NULL || feedback_source == NULL ||
        command_ops == NULL || feedback_ops == NULL || tuning == NULL) {
        return false;
    }

    memset(&actuator_config, 0, sizeof(actuator_config));
    actuator_config.motor_instance_ = motor_instance;
    actuator_config.feedback_source_ = feedback_source;
    actuator_config.angle_pid_config_ = tuning->angle_pid_config_;
    actuator_config.angular_velocity_pid_config_ = tuning->velocity_pid_config_;
    actuator_config.command_ops_ = command_ops;
    actuator_config.feedback_ops_ = feedback_ops;
    actuator_config.control_loop_ = tuning->control_loop_;
    actuator_config.command_type_ = tuning->command_type_;
    actuator_config.feedback_side_ = tuning->feedback_side_;
    actuator_config.command_sign_ = tuning->command_sign_;
    actuator_config.feedback_sign_ = tuning->feedback_sign_;

    return moduleMotorActuatorInit(actuator_out, &actuator_config);
}

static bool appChassisPlatformAppendRMDV2X6Actuator(appChassisPlatformContext_t *context,
                                                    const appChassisPlatformRMDV2X6Spec_t *spec)
{
    motorRMDV2X6Config_t motor_config;
    motorRMDV2X6Instance_t *motor_instance;
    moduleMotorActuator_t *actuator;

    if (context == NULL || spec == NULL) {
        return false;
    }

    if (appChassisPlatformAllocateActuatorSlot(context, &actuator) == false) {
        return false;
    }

    memset(&motor_config, 0, sizeof(motor_config));
    motor_config.can_instance_ = bspBoardGetCANInstance(spec->can_id_);
    motor_config.motor_id_ = spec->motor_id_;
    motor_config.reduction_ratio_ = spec->reduction_ratio_;
    motor_config.fb_abs_angle_high_accuracy_timeout_us_ = spec->fb_abs_angle_high_accuracy_timeout_us_;
    motor_config.abs_time_us_callback_ = bspDWTGetAbsTimeUs;
    motor_config.name_ = spec->name_;

    if (motor_config.can_instance_ == NULL) {
        return false;
    }

    motor_instance = motorRMDV2X6InstanceInit(&motor_config);
    if (motor_instance == NULL) {
        return false;
    }

    if (appChassisPlatformBuildActuator(actuator,
                                        motor_instance,
                                        motor_instance,
                                        adapterGetRMDV2X6CommandOps(),
                                        adapterGetRMDV2X6FeedbackOps(),
                                        &spec->tuning_) == false) {
        return false;
    }

    return appChassisPlatformAppendActuatorEntry(context, spec->name_, actuator);
}

static bool appChassisPlatformAppendDJIM3508Actuator(appChassisPlatformContext_t *context,
                                                     const appChassisPlatformDJIM3508Spec_t *spec)
{
    motorDJIM3508Config_t motor_config;
    motorDJIM3508Instance_t *motor_instance;
    moduleMotorActuator_t *actuator;

    if (context == NULL || spec == NULL) {
        return false;
    }

    if (appChassisPlatformAllocateActuatorSlot(context, &actuator) == false) {
        return false;
    }

    memset(&motor_config, 0, sizeof(motor_config));
    motor_config.can_instance_ = bspBoardGetCANInstance(spec->can_id_);
    motor_config.motor_id_ = spec->motor_id_;
    motor_config.reduction_ratio_ = spec->reduction_ratio_;
    motor_config.abs_time_us_callback_ = bspDWTGetAbsTimeUs;
    motor_config.name_ = spec->name_;

    if (motor_config.can_instance_ == NULL) {
        return false;
    }

    motor_instance = motorDJIM3508InstanceInit(&motor_config);
    if (motor_instance == NULL) {
        return false;
    }

    if (appChassisPlatformBuildActuator(actuator,
                                        motor_instance,
                                        motor_instance,
                                        adapterGetDJIM3508CommandOps(),
                                        adapterGetDJIM3508FeedbackOps(),
                                        &spec->tuning_) == false) {
        return false;
    }

    return appChassisPlatformAppendActuatorEntry(context, spec->name_, actuator);
}

static bool appChassisPlatformStartCANBus(bspCANId_e can_id)
{
    bspCANInstance_t *can_instance;
    bspCANStatus_e can_status;

    can_instance = bspBoardGetCANInstance(can_id);
    if (can_instance == NULL) {
        return false;
    }

    can_status = bspCANSetFilter(can_instance);
    if (can_status != BSP_CAN_OK) {
        return false;
    }

    can_status = bspCANStart(can_instance);
    if (can_status != BSP_CAN_OK) {
        return false;
    }

    return true;
}

static bool appChassisPlatformStartRequiredCANBuses(void)
{
    bool can_started[BSP_CAN_MAX] = {0};
    uint32_t i;

    for (i = 0U; i < APP_CHASSIS_PLATFORM_RMD_V2_X6_SPEC_COUNT; i++) {
        bspCANId_e can_id = app_chassis_platform_rmd_v2_x6_spec_list[i].can_id_;

        if (can_started[can_id] == false) {
            if (appChassisPlatformStartCANBus(can_id) == false) {
                return false;
            }
            can_started[can_id] = true;
        }
    }

#if APP_CHASSIS_PLATFORM_DJI_M3508_SPEC_COUNT > 0U
    for (i = 0U; i < APP_CHASSIS_PLATFORM_DJI_M3508_SPEC_COUNT; i++) {
        bspCANId_e can_id = app_chassis_platform_dji_m3508_spec_list[i].can_id_;

        if (can_started[can_id] == false) {
            if (appChassisPlatformStartCANBus(can_id) == false) {
                return false;
            }
            can_started[can_id] = true;
        }
    }
#endif

    return true;
}

static bool appChassisPlatformInitAllRMDV2X6Actuators(appChassisPlatformContext_t *context)
{
    uint32_t i;

    for (i = 0U; i < APP_CHASSIS_PLATFORM_RMD_V2_X6_SPEC_COUNT; i++) {
        if (appChassisPlatformAppendRMDV2X6Actuator(context,
                                                    &app_chassis_platform_rmd_v2_x6_spec_list[i]) == false) {
            return false;
        }
    }

    return true;
}

static bool appChassisPlatformInitAllDJIM3508Actuators(appChassisPlatformContext_t *context)
{
#if APP_CHASSIS_PLATFORM_DJI_M3508_SPEC_COUNT > 0U
    uint32_t i;

    for (i = 0U; i < APP_CHASSIS_PLATFORM_DJI_M3508_SPEC_COUNT; i++) {
        if (appChassisPlatformAppendDJIM3508Actuator(context,
                                                     &app_chassis_platform_dji_m3508_spec_list[i]) == false) {
            return false;
        }
    }
#else
    (void)context;
#endif

    return true;
}

bool appChassisPlatformInit(appChassisPlatform_t *instance)
{
    appChassisPlatformContext_t *context = &app_chassis_platform_context_;

    if (instance == NULL) {
        return false;
    }

    memset(instance, 0, sizeof(*instance));
    memset(context, 0, sizeof(*context));

    if (appChassisPlatformInitAllRMDV2X6Actuators(context) == false) {
        return false;
    }

    if (appChassisPlatformInitAllDJIM3508Actuators(context) == false) {
        return false;
    }

    if (appChassisPlatformStartRequiredCANBuses() == false) {
        return false;
    }

    instance->context_ = context;

    return true;
}

uint32_t appChassisPlatformGetActuatorCount(const appChassisPlatform_t *instance)
{
    if (instance == NULL || instance->context_ == NULL) {
        return 0U;
    }

    return instance->context_->actuator_count_;
}

moduleMotorActuator_t *appChassisPlatformGetActuatorByIndex(const appChassisPlatform_t *instance, uint32_t index)
{
    if (instance == NULL || instance->context_ == NULL) {
        return NULL;
    }

    if (index >= instance->context_->actuator_count_) {
        return NULL;
    }

    return instance->context_->actuator_entry_[index].actuator_;
}

const char *appChassisPlatformGetActuatorNameByIndex(const appChassisPlatform_t *instance, uint32_t index)
{
    if (instance == NULL || instance->context_ == NULL) {
        return NULL;
    }

    if (index >= instance->context_->actuator_count_) {
        return NULL;
    }

    return instance->context_->actuator_entry_[index].name_;
}

moduleMotorActuator_t *appChassisPlatformFindActuatorByName(const appChassisPlatform_t *instance, const char *name)
{
    uint32_t i;

    if (instance == NULL || instance->context_ == NULL || name == NULL) {
        return NULL;
    }

    for (i = 0U; i < instance->context_->actuator_count_; i++) {
        const char *actuator_name = instance->context_->actuator_entry_[i].name_;
        moduleMotorActuator_t *actuator = instance->context_->actuator_entry_[i].actuator_;

        if (actuator_name == NULL || actuator == NULL) {
            continue;
        }

        if (strcmp(actuator_name, name) == 0) {
            return actuator;
        }
    }

    return NULL;
}
