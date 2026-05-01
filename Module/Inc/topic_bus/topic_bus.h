#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct topic moduleTopic_t;

typedef enum
{
    MODULE_TOPIC_WAIT_NONE = 0, // 不需要通知
    MODULE_TOPIC_WAIT_TASK_NOTIFY,
    MODULE_TOPIC_WAIT_SEMAPHORE,
} moduleTopicWaitBackend_e;

typedef struct topic_subscription 
{
    const moduleTopic_t *topic_; // 指向订阅的topic
    uint32_t next_subscribe_seq_num_; // 下一个需要订阅的消息序号
    uint32_t lost_count_; // 丢失的消息总数

    moduleTopicWaitBackend_e wait_backend_;
    union {
        TaskHandle_t wait_task_; // 等待topic更新的任务
        SemaphoreHandle_t wait_sem_; // 等待的二值信号量
    } wait_handle_;
} moduleTopicSubscription_t; // 订阅者结构体

bool moduleTopicBusPublish(moduleTopic_t *topic, const void *message);
bool moduleTopicBusSubscribe(moduleTopic_t *topic, moduleTopicSubscription_t *subscriber);
// 检查该话题是否有更新
bool moduleTopicBusUpdated(const moduleTopicSubscription_t *subscriber);
// 从话题拷贝数据
bool moduleTopicBusCopy(moduleTopicSubscription_t *subscriber, void *message_out);
// 等待话题发布新数据，请注意，只有当有新数据时才返回true
bool moduleTopicBusWait(moduleTopicSubscription_t *subscriber, TickType_t timeout_ticks);