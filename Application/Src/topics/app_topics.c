#include <stdint.h>
#include <string.h>

#include "app_topics.h"
#include "msg_rc_command.h"
#include "msg_ins.h"
#include "msg_topic_bus_test.h"
#include "topic_bus.h"
#include "topic_bus_private.h"

// static uint8_t rc_command_buffer_[MSG_RC_COMMAND_TOPIC_DEPTH * sizeof(msgRCCommand_t)] = {0};
// static moduleTopicSubscription_t *rc_command_waiter_list_[MSG_RC_COMMAND_TOPIC_MAX_WAITER] = {NULL};

static uint8_t ins_buffer_[MSG_INS_TOPIC_DEPTH * sizeof(msgINS_t)] = {0};
static moduleTopicSubscription_t *ins_waiter_list_[MSG_INS_TOPIC_MAX_WAITER] = {NULL};

static uint8_t topic_bus_test_cmd_buffer_[MSG_TOPIC_BUS_TEST_CMD_TOPIC_DEPTH * sizeof(msgTopicBusCmdTest_t)] = {0};
static moduleTopicSubscription_t *topic_bus_test_cmd_waiter_list_[MSG_TOPIC_BUS_TEST_CMD_TOPIC_MAX_WAITER] = {NULL};

static uint8_t topic_bus_test_state_buffer_[MSG_TOPIC_BUS_TEST_STATE_TOPIC_DEPTH * sizeof(msgTopicBusStateTest_t)] = {0};
static moduleTopicSubscription_t *topic_bus_test_state_waiter_list_[MSG_TOPIC_BUS_TEST_STATE_TOPIC_MAX_WAITER] = {NULL};

static uint8_t topic_bus_test_state_aux_buffer_[MSG_TOPIC_BUS_TEST_STATE_AUX_TOPIC_DEPTH * sizeof(msgTopicBusStateTest_t)] = {0};
static moduleTopicSubscription_t *topic_bus_test_state_aux_waiter_list_[MSG_TOPIC_BUS_TEST_STATE_AUX_TOPIC_MAX_WAITER] = {NULL};

// 全部为编译期已知
static moduleTopic_t topic_memory_[APP_TOPICS_MAX] = {
    // [APP_TOPICS_RC_COMMAND] = {
    //     .msg_size_ = sizeof(msgRCCommand_t),
    //     .depth_ = MSG_RC_COMMAND_TOPIC_DEPTH,
    //     .buffer_ = rc_command_buffer_,
    //     .publish_seq_num_ = 0U,
    //     .publish_index_ = 0U,
    //     .waiter_list_ = rc_command_waiter_list_,
    //     .waiter_count_ = 0U,
    //     .waiter_list_capacity_ = MSG_RC_COMMAND_TOPIC_MAX_WAITER,
    //     .topic_name_ = MSG_RC_COMMAND_TOPIC_NAME,
    // },
    [APP_TOPICS_INS] = {
        .msg_size_ = sizeof(msgINS_t),
        .depth_ = MSG_INS_TOPIC_DEPTH,
        .buffer_ = ins_buffer_,
        .publish_seq_num_ = 0U,
        .publish_index_ = 0U,
        .waiter_list_ = ins_waiter_list_,
        .waiter_count_ = 0U,
        .waiter_list_capacity_ = MSG_INS_TOPIC_MAX_WAITER,
        .topic_name_ = MSG_INS_TOPIC_NAME,
    },
    [APP_TOPICS_TOPIC_BUS_TEST_CMD] = {
        .msg_size_ = sizeof(msgTopicBusCmdTest_t),
        .depth_ = MSG_TOPIC_BUS_TEST_CMD_TOPIC_DEPTH,
        .buffer_ = topic_bus_test_cmd_buffer_,
        .publish_seq_num_ = 0U,
        .publish_index_ = 0U,
        .waiter_list_ = topic_bus_test_cmd_waiter_list_,
        .waiter_count_ = 0U,
        .waiter_list_capacity_ = MSG_TOPIC_BUS_TEST_CMD_TOPIC_MAX_WAITER,
        .topic_name_ = MSG_TOPIC_BUS_TEST_CMD_TOPIC_NAME,
    },
    [APP_TOPICS_TOPIC_BUS_TEST_STATE] = {
        .msg_size_ = sizeof(msgTopicBusStateTest_t),
        .depth_ = MSG_TOPIC_BUS_TEST_STATE_TOPIC_DEPTH,
        .buffer_ = topic_bus_test_state_buffer_,
        .publish_seq_num_ = 0U,
        .publish_index_ = 0U,
        .waiter_list_ = topic_bus_test_state_waiter_list_,
        .waiter_count_ = 0U,
        .waiter_list_capacity_ = MSG_TOPIC_BUS_TEST_STATE_TOPIC_MAX_WAITER,
        .topic_name_ = MSG_TOPIC_BUS_TEST_STATE_TOPIC_NAME,
    },
    [APP_TOPICS_TOPIC_BUS_TEST_STATE_AUX] = {
        .msg_size_ = sizeof(msgTopicBusStateTest_t),
        .depth_ = MSG_TOPIC_BUS_TEST_STATE_AUX_TOPIC_DEPTH,
        .buffer_ = topic_bus_test_state_aux_buffer_,
        .publish_seq_num_ = 0U,
        .publish_index_ = 0U,
        .waiter_list_ = topic_bus_test_state_aux_waiter_list_,
        .waiter_count_ = 0U,
        .waiter_list_capacity_ = MSG_TOPIC_BUS_TEST_STATE_AUX_TOPIC_MAX_WAITER,
        .topic_name_ = MSG_TOPIC_BUS_TEST_STATE_AUX_TOPIC_NAME,
    },
};

moduleTopic_t *appTopicsGet(appTopics_e topic_id)
{
    if (topic_id < 0 || topic_id >= APP_TOPICS_MAX) {
        return NULL;
    }

    return &topic_memory_[topic_id];
}
