#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "topic_bus.h"

typedef struct topic_subscription moduleTopicSubscription_t;

typedef struct topic 
{
    uint16_t msg_size_; // 这个topic上每条消息的大小/字节数
    uint8_t depth_; // 消息深度
    uint8_t *buffer_; // 该topic绑定的消息缓冲数组
    uint32_t publish_seq_num_; // 写入到这个topic的消息的序号/总数，从0开始
    uint16_t publish_index_; // 写入消息的索引

    moduleTopicSubscription_t **waiter_list_; // 该topic订阅者(需要通知的，因为有的订阅者不需要通知)指针列表的指针
    uint8_t waiter_count_;
    uint8_t waiter_list_capacity_;

    const char *topic_name_;
} moduleTopic_t;