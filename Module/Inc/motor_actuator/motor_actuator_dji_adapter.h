#pragma once

#include "motor_actuator.h"
#include "motor_actuator_group.h"

// 获取虚函数表
const moduleMotorActuatorCommandOps_t *adapterGetDJIM3508CommandOps(void);
const moduleMotorActuatorFeedbackOps_t *adapterGetDJIM3508FeedbackOps(void);
const moduleMotorActuatorGroupCommandOps_t *adapterGetDJIM3508GroupCommandOps(void);