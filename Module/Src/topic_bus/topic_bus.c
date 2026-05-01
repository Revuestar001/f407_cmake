#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "topic_bus.h"
#include "topic_bus_private.h"

static bool checkTopicIsValid(const moduleTopic_t *topic)
{
    if (topic == NULL) {
        return false;
    }

    if (topic->buffer_ == NULL || topic->msg_size_ == 0U || topic->depth_ == 0U) {
        return false;
    }

    return true;
}

bool moduleTopicBusPublish(moduleTopic_t *topic, const void *message)
{
    if (topic == NULL || message == NULL) {
        return false;
    }

    if (checkTopicIsValid(topic) == false) {
        return false;
    }

    uint8_t waiter_count_snapshot;
    uint16_t publish_index = topic->publish_index_;
    // 计算写入偏移量
    uint8_t *buffer_slot = topic->buffer_ + ((size_t)publish_index * topic->msg_size_);

    // 进入临界区，这里暂时只支持任务间通信，因此只关闭freertos管理的中断
    taskENTER_CRITICAL();
    // 拷贝数据到topic缓冲区指定位置
    memcpy(buffer_slot, message, topic->msg_size_);

    topic->publish_seq_num_++; // 下一条消息的序号
    topic->publish_index_ = (publish_index + 1U) % topic->depth_;

    waiter_count_snapshot = topic->waiter_count_;
    taskEXIT_CRITICAL();

    // 通知需要的订阅者
    for (size_t i = 0; i < waiter_count_snapshot; i ++) {
        TaskHandle_t task_to_notify = NULL;
        SemaphoreHandle_t semphr_to_notify = NULL;

        // 临界区内获取需要被通知的任务句柄/信号量
        taskENTER_CRITICAL();
        moduleTopicSubscription_t *waiter = topic->waiter_list_[i];
        if (waiter != NULL) {
            if (waiter->wait_backend_ == MODULE_TOPIC_WAIT_TASK_NOTIFY) {
                task_to_notify = waiter->wait_handle_.wait_task_;
            } else if (waiter->wait_backend_ == MODULE_TOPIC_WAIT_SEMAPHORE) {
                semphr_to_notify = waiter->wait_handle_.wait_sem_;
            }
        }
        taskEXIT_CRITICAL();

        if (task_to_notify != NULL) {
            xTaskNotifyGive(task_to_notify);
            continue;
        }

        if (semphr_to_notify != NULL) {
            xSemaphoreGive(semphr_to_notify);
            continue;
        }
    }

    return true;
}

bool moduleTopicBusSubscribe(moduleTopic_t *topic, moduleTopicSubscription_t *subscriber)
{
    if (topic == NULL || subscriber == NULL) {
        return false;
    }

    if (checkTopicIsValid(topic) == false) {
        return false;
    }

    // 不允许订阅一个话题后通过该API订阅另一个话题
    if (subscriber->topic_ != NULL && subscriber->topic_ != topic) {
        return false;
    }

    taskENTER_CRITICAL();

    if (subscriber->wait_backend_ != MODULE_TOPIC_WAIT_NONE) {
        if (topic->waiter_list_ == NULL || topic->waiter_list_capacity_ == 0U) {
            taskEXIT_CRITICAL();
            return false;
        }

        if (topic->waiter_count_ >= topic->waiter_list_capacity_) {
            // 超出通知列表容量
            taskEXIT_CRITICAL();
            return false;
        }

        for (uint8_t i = 0; i < topic->waiter_count_; i++) {
            if (topic->waiter_list_[i] == subscriber) {
                // 需要通知的订阅者重复
                taskEXIT_CRITICAL();
                return false;
            }
        }

        if (subscriber->wait_backend_ == MODULE_TOPIC_WAIT_TASK_NOTIFY) {
            if (subscriber->wait_handle_.wait_task_ == NULL) {
                taskEXIT_CRITICAL();
                return false;
            }
        } else if (subscriber->wait_backend_ == MODULE_TOPIC_WAIT_SEMAPHORE) {
            if (subscriber->wait_handle_.wait_sem_ == NULL) {
                taskEXIT_CRITICAL();
                return false;
            }
        } else {
            taskEXIT_CRITICAL();
            return false;
        }

        topic->waiter_list_[topic->waiter_count_] = subscriber;
        topic->waiter_count_++;
    }

    subscriber->topic_ = topic;
    subscriber->next_subscribe_seq_num_ = topic->publish_seq_num_; // 下一个需要订阅的消息序号
    subscriber->lost_count_ = 0U;
    taskEXIT_CRITICAL();

    return true;
}

// 检查该话题是否有更新
bool moduleTopicBusUpdated(const moduleTopicSubscription_t *subscriber)
{
    if (subscriber == NULL) {
        return false;
    }

    if (checkTopicIsValid(subscriber->topic_) == false) {
        return false;
    }

    return subscriber->next_subscribe_seq_num_ < subscriber->topic_->publish_seq_num_;
}

// 从话题拷贝数据
bool moduleTopicBusCopy(moduleTopicSubscription_t *subscriber, void *message_out)
{
    if (subscriber == NULL || message_out == NULL) {
        return false;
    }

    if (checkTopicIsValid(subscriber->topic_) == false) {
        return false;
    }

    taskENTER_CRITICAL();
    if (subscriber->next_subscribe_seq_num_ >= subscriber->topic_->publish_seq_num_) {
        // 没有新数据，可以拷贝也可以不拷贝返回false，这里先使用后者
        taskEXIT_CRITICAL();
        return false;
    }

    uint32_t read_seq_num;
    // 计算最旧的能读到的消息序号
    uint32_t oldest_seq_num = (subscriber->topic_->publish_seq_num_ > subscriber->topic_->depth_) ? (subscriber->topic_->publish_seq_num_ - subscriber->topic_->depth_) : 0U;

    if (oldest_seq_num > subscriber->next_subscribe_seq_num_) {
        // 落后太多
        // 记录丢失消息数
        subscriber->lost_count_ +=  oldest_seq_num - subscriber->next_subscribe_seq_num_;
        // 最旧的能读到的消息序号
        read_seq_num = oldest_seq_num;
    } else {
        read_seq_num = subscriber->next_subscribe_seq_num_;
    }

    uint16_t read_index = read_seq_num % subscriber->topic_->depth_;
    uint8_t *buffer_slot = subscriber->topic_->buffer_ + (size_t)read_index * subscriber->topic_->msg_size_;

    memcpy(message_out, buffer_slot, subscriber->topic_->msg_size_);
    taskEXIT_CRITICAL();

    // 下一个要读的消息序号就是本次读的消息序号的下一个
    subscriber->next_subscribe_seq_num_ = read_seq_num + 1U;

    return true;
}

// 等待话题发布新数据，请注意，只有当有新数据时才返回true
bool moduleTopicBusWait(moduleTopicSubscription_t *subscriber, TickType_t timeout_ticks)
{
    if (subscriber == NULL) {
        return false;
    }

    if (checkTopicIsValid(subscriber->topic_) == false) {
        return false;
    }

    // 先检查该话题是否有更新
    if (moduleTopicBusUpdated(subscriber) == true) {
        return true;
    }

    switch (subscriber->wait_backend_) {
        case MODULE_TOPIC_WAIT_TASK_NOTIFY:
            if (subscriber->wait_handle_.wait_task_ == NULL) {
                return false;
            }

            if (subscriber->wait_handle_.wait_task_ != xTaskGetCurrentTaskHandle()) {
                // 如果当前任务不是订阅者等待任务，直接返回
                return false;
            }
            // 还没有更新，开始按照超时时间等待
            if (ulTaskNotifyTake(pdTRUE, timeout_ticks) == 0U) {
                // 获取任务通知超时
                return false;
            }
            break;
        case MODULE_TOPIC_WAIT_SEMAPHORE:
            if (subscriber->wait_handle_.wait_sem_ == NULL) {
                return false;
            }
            // 还没有更新，开始按照超时时间等待
            if (xSemaphoreTake(subscriber->wait_handle_.wait_sem_, timeout_ticks) != pdTRUE) {
                // 获取任务通知超时
                return false;
            }
            break;
        case MODULE_TOPIC_WAIT_NONE:
        default:
            return false;
    }

    // 收到通知，再检查一次是否更新
    return moduleTopicBusUpdated(subscriber);
}