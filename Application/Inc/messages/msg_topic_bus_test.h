#pragma once

#include <stdint.h>

typedef struct msg_topic_bus_cmd_test
{
    uint32_t publish_seq_;
    uint32_t payload_pattern_;
    uint32_t publish_tick_;
    uint32_t publisher_tag_;
} msgTopicBusCmdTest_t;

typedef struct msg_topic_bus_state_test
{
    uint32_t publish_seq_;
    uint32_t payload_pattern_;
    uint32_t publish_tick_;
    uint32_t publisher_tag_;
    uint32_t payload_words_[28];
} msgTopicBusStateTest_t;

_Static_assert(sizeof(msgTopicBusCmdTest_t) == 16U, "msgTopicBusCmdTest_t size mismatch");
_Static_assert(sizeof(msgTopicBusStateTest_t) == 128U, "msgTopicBusStateTest_t must be 128 bytes");
