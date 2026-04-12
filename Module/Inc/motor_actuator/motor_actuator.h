#pragma once

#include "pid.h"

typedef struct module_motor_actuator_config
{
    void *motor_instance_;

    algorithmPIDConfig_t angle_pid_config_;
    algorithmPIDConfig_t angular_velocity_pid_config_;
    algorithmPIDConfig_t current_pid_config_;
} moduleMotorActuatorConfig_t;

typedef struct module_motor_actuator
{
    void *motor_instance_;

    algorithmPID_t angle_pid_;
    algorithmPID_t angular_velocity_pid_;
    algorithmPID_t current_pid_;
} moduleMotorActuator_t;