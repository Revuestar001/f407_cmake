#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "motor_actuator_group.h"
#include "motor_actuator.h"
#include "dji_m3508.h"
#include "motor_def.h"

// true表示两个commit group相同，两个null也算相同
static bool compareCommitGroup(const moduleMotorActuatorCommitGroup_t *group1, const moduleMotorActuatorCommitGroup_t *group2)
{
    if (group1 == NULL && group2 == NULL) {
        return true;
    } else if ((group1 == NULL && group2 != NULL) || (group1 != NULL && group2 == NULL)) {
        return false;
    }

    if (group1->bus_ != group2->bus_ || group1->group_id_ != group2->group_id_) {
        return false;
    }

    return true;
}

bool moduleMotorActuatorGroupInit(moduleMotorActuatorGroup_t *instance, moduleMotorActuator_t *motors[MODULE_MOTOR_ACTUATOR_MAX_MOTOR_ONE_GROUP])
{
    if (instance == NULL || motors == NULL) {
        return false;
    }

    memset(instance, 0, sizeof(moduleMotorActuatorGroup_t));

    const moduleMotorActuatorCommitGroup_t *base_commit_group = NULL;

    for (size_t i = 0; i < MODULE_MOTOR_ACTUATOR_MAX_MOTOR_ONE_GROUP; i++) {
        if (motors[i] == NULL) {
            continue;
        }

        if (motors[i]->is_initialized_ == false) {
            // 只要有一个电机未初始化，则group初始化失败
            memset(instance, 0, sizeof(moduleMotorActuatorGroup_t));
            return false;
        }

        if (base_commit_group == NULL) {
            base_commit_group = &motors[i]->commit_group_;
        } else if (compareCommitGroup(base_commit_group, &motors[i]->commit_group_) == false) {
            // 检查到不同 commit group 的电机试图注册到同一个 MotorActuatorGroup， group初始化失败
            memset(instance, 0, sizeof(moduleMotorActuatorGroup_t));
            return false;
        }

        instance->motor_instance_group[i] = motors[i];
    }

    if (base_commit_group == NULL) {
        // 没有有效电机,group初始化失败
        return false;
    }

    return true;
} 

motorStatus_e moduleMotorActuatorGroupCommit(moduleMotorActuatorGroup_t *instance)
{
    if (instance == NULL) {
        return MOTOR_ERROR;
    }

    for (size_t i = 0; i < MODULE_MOTOR_ACTUATOR_MAX_MOTOR_ONE_GROUP; i ++) {
        if (instance->motor_instance_group[i] != NULL) {
            // 这个返回值我觉得比较重要，但是可能这个函数不要用bool比较好
            motorStatus_e motor_status;
            motor_status = motorDJIM3508GroupSendByMotorInstance(instance->motor_instance_group[i]->motor_instance_);
            if (motor_status != MOTOR_OK) {
                return motor_status;
            }
            return MOTOR_OK;
        }
    }

    return MOTOR_ERROR;
}
