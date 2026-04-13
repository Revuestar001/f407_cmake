#include <string.h>

#include "motor_actuator_dji_adapter.h"
#include "motor_actuator.h"
#include "motor_actuator_group.h"
#include "motor_def.h"

#include "dji_m3508.h"

//
// DJI M3508
//
static bool adapterDJIM3508SetCurrentRef(void *motor, float current_ref)
{
    return motorDJIM3508SetCurrentRef((motorDJIM3508Instance_t *)motor, current_ref);
}

static bool adapterDJIM3508SetWorkStatus(void *motor, motorWorkStatus_e work_status)
{
    return motorDJIM3508SetWorkStatus((motorDJIM3508Instance_t *)motor, work_status);
}

static bool adapterDJIM3508GetCommitGroup(void *motor, moduleMotorActuatorCommitGroup_t *group_out)
{
    if (group_out == NULL) {
        return false;
    }

    return motorDJIM3508GetCommitGroup((motorDJIM3508Instance_t *)motor, &group_out->bus_, &group_out->group_id_);
}

static motorStatus_e adapterDJIM3508UpdateFeedbackData(void *motor)
{
    return motorDJIM3508UpdateFeedbackData((motorDJIM3508Instance_t *)motor);
}

static motorStatus_e adapterDJIM3508GetFeedbackData(void *motor, motorFeedBackData_t *data_out)
{
    if (data_out == NULL) {
        return MOTOR_ERROR;
    }

    return motorDJIM3508GetFeedbackData((motorDJIM3508Instance_t *)motor, data_out);
}

static motorStatus_e adapterDJIM3508GroupSendByMotorInstance(const void *motor)
{
    return motorDJIM3508GroupSendByMotorInstance((motorDJIM3508Instance_t *)motor);
}

// 定义command虚函数表，注意是const
static const moduleMotorActuatorCommandOps_t moduleMotorActuatorDJIM3508CommandOps = {
    .set_effort_ref_ = adapterDJIM3508SetCurrentRef, // 实际上是电流值
    .set_work_status_ = adapterDJIM3508SetWorkStatus,
    .get_commit_group_ = adapterDJIM3508GetCommitGroup,
    // 不支持单电机commit
};

static const moduleMotorActuatorFeedbackOps_t moduleMotorActuatorDJIM3508FeedbackOps = {
    .update_feedback_ = adapterDJIM3508UpdateFeedbackData,
    .get_feedback_ = adapterDJIM3508GetFeedbackData,
};

static const moduleMotorActuatorGroupCommandOps_t moduleMotorActuatorGroupDJIM3508CommandOps = {
    .group_commit_ = adapterDJIM3508GroupSendByMotorInstance,
};

const moduleMotorActuatorCommandOps_t *adapterGetDJIM3508CommandOps(void)
{
    return &moduleMotorActuatorDJIM3508CommandOps;
}

const moduleMotorActuatorFeedbackOps_t *adapterGetDJIM3508FeedbackOps(void)
{
    return &moduleMotorActuatorDJIM3508FeedbackOps;
}

const moduleMotorActuatorGroupCommandOps_t *adapterGetDJIM3508GroupCommandOps(void)
{
    return &moduleMotorActuatorGroupDJIM3508CommandOps;
}