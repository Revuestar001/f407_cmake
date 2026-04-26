#include <string.h>

#include "motor_actuator_myactuator_adapter.h"
#include "motor_actuator.h"
#include "motor_def.h"

#include "rmd_v2_x6.h"

//
// RMD V2 X6
//
static bool adapterRMDV2X6SetEffortRef(void *motor, float effort_ref)
{
    return motorRMDV2X6SetEffortRef((motorRMDV2X6Instance_t *)motor, effort_ref);
}

static bool adapterRMDV2X6SetWorkStatus(void *motor, motorWorkStatus_e work_status)
{
    return motorRMDV2X6SetWorkStatus((motorRMDV2X6Instance_t *)motor, work_status);
}

static motorStatus_e adapterRMDV2X6SendEffortCommand(const void *motor)
{
    return motorRMDV2X6SendEffortCommand((motorRMDV2X6Instance_t *)motor);
}

static motorStatus_e adapterRMDV2X6UpdateFeedbackData(void *motor)
{
    return motorRMDV2X6UpdateFeedbackData((motorRMDV2X6Instance_t *)motor);
}

static motorStatus_e adapterRMDV2X6GetFeedbackData(void *motor, motorFeedBackData_t *data_out)
{
    if (data_out == NULL) {
        return MOTOR_ERROR;
    }

    return motorRMDV2X6GetFeedbackData((motorRMDV2X6Instance_t *)motor, data_out);
}

// 定义command虚函数表，注意是const
static const moduleMotorActuatorCommandOps_t moduleMotorActuatorRMDV2X6CommandOps = {
    .set_effort_ref_ = adapterRMDV2X6SetEffortRef,
    .set_work_status_ = adapterRMDV2X6SetWorkStatus,
    // 支持单电机commit
    .commit_command_ = adapterRMDV2X6SendEffortCommand,
};

static const moduleMotorActuatorFeedbackOps_t moduleMotorActuatorRMDV2X6FeedbackOps = {
    .update_feedback_ = adapterRMDV2X6UpdateFeedbackData,
    .get_feedback_ = adapterRMDV2X6GetFeedbackData,
};

const moduleMotorActuatorCommandOps_t *adapterGetRMDV2X6CommandOps(void)
{
    return &moduleMotorActuatorRMDV2X6CommandOps;
}

const moduleMotorActuatorFeedbackOps_t *adapterGetRMDV2X6FeedbackOps(void)
{
    return &moduleMotorActuatorRMDV2X6FeedbackOps;
}