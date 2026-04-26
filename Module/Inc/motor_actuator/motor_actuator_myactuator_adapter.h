#pragma once

#include "motor_actuator.h"

// 获取虚函数表
const moduleMotorActuatorCommandOps_t *adapterGetRMDV2X6CommandOps(void);
const moduleMotorActuatorFeedbackOps_t *adapterGetRMDV2X6FeedbackOps(void);