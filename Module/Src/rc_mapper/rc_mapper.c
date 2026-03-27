#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "rc_mapper.h"

static int16_t moduleRCMapperLimitStickValue(int32_t value)
{
    if (value > MODULE_RC_MAPPER_STICK_MAX_VALUE) {
        return MODULE_RC_MAPPER_STICK_MAX_VALUE;
    }

    if (value < MODULE_RC_MAPPER_STICK_MIN_VALUE) {
        return MODULE_RC_MAPPER_STICK_MIN_VALUE;
    }

    return (int16_t)value;
}

static int16_t moduleRCMapperMapChannelUsToStickValue(uint16_t channel_us)
{
    int32_t stick_value = ((int32_t)channel_us - (int32_t)MODULE_RC_MAPPER_CHANNEL_US_MID_VALUE) *
                          MODULE_RC_MAPPER_STICK_MAX_VALUE /
                          (int32_t)MODULE_RC_MAPPER_CHANNEL_US_HALF_RANGE;

    stick_value = moduleRCMapperLimitStickValue(stick_value);

    if (stick_value < MODULE_RC_MAPPER_STICK_DEADBAND_VALUE &&
        stick_value > -MODULE_RC_MAPPER_STICK_DEADBAND_VALUE) {
        stick_value = 0;
    }

    return (int16_t)stick_value;
}

static moduleRCMapperSwitchState_e moduleRCMapperMapChannelUsToSwitchState(uint16_t channel_us)
{
    if (channel_us < MODULE_RC_MAPPER_SWITCH_LOW_THRESHOLD) {
        return MODULE_RC_MAPPER_SWITCH_DOWN;
    }

    if (channel_us > MODULE_RC_MAPPER_SWITCH_HIGH_THRESHOLD) {
        return MODULE_RC_MAPPER_SWITCH_UP;
    }

    return MODULE_RC_MAPPER_SWITCH_MID;
}

static moduleRCMapperSwitchEdge_e moduleRCMapperGetSwitchEdgeFromState(moduleRCMapperSwitchState_e last_state,
                                                                       moduleRCMapperSwitchState_e current_state)
{
    if (current_state > last_state) {
        return MODULE_RC_MAPPER_SWITCH_EDGE_RISING;
    }

    if (current_state < last_state) {
        return MODULE_RC_MAPPER_SWITCH_EDGE_FALLING;
    }

    return MODULE_RC_MAPPER_SWITCH_EDGE_NONE;
}

void moduleRCMapperInit(moduleRCMapper_t *instance)
{
    if (instance == NULL) {
        return;
    }

    memset(instance, 0, sizeof(moduleRCMapper_t));
}

bool moduleRCMapperUpdateFromSBUSFrame(moduleRCMapper_t *instance, const protocolSBUSDataFrame_t *sbus_frame)
{
    if (instance == NULL || sbus_frame == NULL) {
        return false;
    }

    instance->stick_right_x_ = moduleRCMapperMapChannelUsToStickValue(sbus_frame->channel_data_us_[0]);
    instance->stick_right_y_ = moduleRCMapperMapChannelUsToStickValue(sbus_frame->channel_data_us_[1]);
    instance->stick_left_y_ = moduleRCMapperMapChannelUsToStickValue(sbus_frame->channel_data_us_[2]);
    instance->stick_left_x_ = moduleRCMapperMapChannelUsToStickValue(sbus_frame->channel_data_us_[3]);

    instance->switch_left_.last_state_ = instance->switch_left_.state_;
    instance->switch_right_.last_state_ = instance->switch_right_.state_;

    instance->switch_left_.state_ = moduleRCMapperMapChannelUsToSwitchState(sbus_frame->channel_data_us_[4]);
    instance->switch_right_.state_ = moduleRCMapperMapChannelUsToSwitchState(sbus_frame->channel_data_us_[5]);
    instance->switch_left_.edge_ = moduleRCMapperGetSwitchEdgeFromState(instance->switch_left_.last_state_, instance->switch_left_.state_);
    instance->switch_right_.edge_ = moduleRCMapperGetSwitchEdgeFromState(instance->switch_right_.last_state_, instance->switch_right_.state_);

    instance->frame_lost_flag_ = sbus_frame->frame_lost_flag_;
    instance->failsafe_activate_flag_ = sbus_frame->failsafe_activate_flag_;

    return true;
}

bool moduleRCMapperSwitchStateChanged(const moduleRCSwitch_t *switch_instance)
{
    if (switch_instance == NULL) {
        return false;
    }

    return switch_instance->state_ != switch_instance->last_state_;
}

moduleRCMapperSwitchEdge_e moduleRCMapperSwitchGetEdge(const moduleRCSwitch_t *switch_instance)
{
    if (switch_instance == NULL) {
        return MODULE_RC_MAPPER_SWITCH_EDGE_NONE;
    }

    return switch_instance->edge_;
}

bool moduleRCMapperSwitchEdgeMatched(const moduleRCSwitch_t *switch_instance, moduleRCMapperSwitchEdge_e edge)
{
    if (switch_instance == NULL) {
        return false;
    }

    return (switch_instance->edge_ & edge) != 0U;
}
