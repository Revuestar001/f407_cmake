#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_topic_bus_test.h"
#include "app_topics.h"
#include "msg_topic_bus_test.h"
#include "topic_bus.h"

#define APP_TOPIC_BUS_TEST_CMD_PERIOD_MS 20U
#define APP_TOPIC_BUS_TEST_STATE_PERIOD_MS 1U
#define APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS 200U
#define APP_TOPIC_BUS_TEST_CMD_FAST_TARGET 40U
#define APP_TOPIC_BUS_TEST_CMD_SLOW_POLL_PERIOD_MS 100U
#define APP_TOPIC_BUS_TEST_CMD_SLOW_MIN_COPY_COUNT 5U
#define APP_TOPIC_BUS_TEST_STATE_PHASE1_TARGET 1000U
#define APP_TOPIC_BUS_TEST_STATE_FORCED_LAG_MS 20U
#define APP_TOPIC_BUS_TEST_PAYLOAD_PATTERN_BASE 0xA55A5AA5UL
#define APP_TOPIC_BUS_TEST_CMD_TAG 0x434D4421UL
#define APP_TOPIC_BUS_TEST_STATE_TAG 0x53544154UL

typedef enum app_topic_bus_test_failure
{
    APP_TOPIC_BUS_TEST_FAILURE_NONE = 0U,
    APP_TOPIC_BUS_TEST_FAILURE_TOPIC_NOT_FOUND = 1U,
    APP_TOPIC_BUS_TEST_FAILURE_SEMAPHORE_CREATE = 2U,
    APP_TOPIC_BUS_TEST_FAILURE_SUBSCRIBE = 3U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_WAIT_TIMEOUT = 4U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_FALSE_WAKEUP = 5U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_PAYLOAD = 6U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_NON_MONOTONIC = 7U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_GAP = 8U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_SLOW_PAYLOAD = 9U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_SLOW_NON_MONOTONIC = 10U,
    APP_TOPIC_BUS_TEST_FAILURE_CMD_SLOW_NO_OVERWRITE = 11U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_WAIT_TIMEOUT = 12U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_FALSE_WAKEUP = 13U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_PAYLOAD = 14U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_NON_MONOTONIC = 15U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_PHASE1_GAP = 16U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_NO_OVERWRITE = 17U,
} appTopicBusTestFailure_e;

typedef enum app_topic_bus_state_phase
{
    APP_TOPIC_BUS_TEST_STATE_PHASE_INIT = 0U,
    APP_TOPIC_BUS_TEST_STATE_PHASE_NO_LOSS = 1U,
    APP_TOPIC_BUS_TEST_STATE_PHASE_FORCE_OVERWRITE = 2U,
    APP_TOPIC_BUS_TEST_STATE_PHASE_DONE = 3U,
} appTopicBusStatePhase_e;

static StaticSemaphore_t topic_bus_test_cmd_fast_sem_struct_;
static StaticSemaphore_t topic_bus_test_state_sem_struct_;

static moduleTopicSubscription_t topic_bus_test_cmd_fast_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_SEMAPHORE,
};
static moduleTopicSubscription_t topic_bus_test_cmd_slow_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_NONE,
};
static moduleTopicSubscription_t topic_bus_test_state_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_SEMAPHORE,
};

volatile appTopicBusTestStats_t g_app_topic_bus_test_stats_ = {0};

static void appTopicBusTestSetFailure(appTopicBusTestFailure_e failure_code)
{
    taskENTER_CRITICAL();
    if (g_app_topic_bus_test_stats_.suite_done_ == 0U) {
        g_app_topic_bus_test_stats_.failure_code_ = (uint32_t)failure_code;
        g_app_topic_bus_test_stats_.suite_done_ = 1U;
        g_app_topic_bus_test_stats_.suite_passed_ = 0U;
    }
    taskEXIT_CRITICAL();
}

static void appTopicBusTestLatchFailure(appTopicBusTestFailure_e failure_code)
{
    appTopicBusTestSetFailure(failure_code);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

static uint32_t appTopicBusTestBuildPattern(uint32_t publish_seq, uint32_t publisher_tag)
{
    return APP_TOPIC_BUS_TEST_PAYLOAD_PATTERN_BASE ^ publish_seq ^ publisher_tag;
}

static uint32_t appTopicBusTestBuildStateWord(uint32_t publish_seq, uint32_t publisher_tag, uint32_t word_index)
{
    return APP_TOPIC_BUS_TEST_PAYLOAD_PATTERN_BASE ^
           publisher_tag ^
           publish_seq ^
           (0x9E3779B9UL * (word_index + 1U));
}

static void appTopicBusTestBuildStateMessage(msgTopicBusStateTest_t *message,
                                             uint32_t publish_seq,
                                             TickType_t publish_tick)
{
    uint32_t i;

    if (message == NULL) {
        return;
    }

    message->publish_seq_ = publish_seq;
    message->payload_pattern_ = appTopicBusTestBuildPattern(publish_seq, APP_TOPIC_BUS_TEST_STATE_TAG);
    message->publish_tick_ = (uint32_t)publish_tick;
    message->publisher_tag_ = APP_TOPIC_BUS_TEST_STATE_TAG;

    for (i = 0U; i < (sizeof(message->payload_words_) / sizeof(message->payload_words_[0])); i++) {
        message->payload_words_[i] = appTopicBusTestBuildStateWord(publish_seq,
                                                                   APP_TOPIC_BUS_TEST_STATE_TAG,
                                                                   i);
    }
}

static bool appTopicBusTestValidateStateMessage(const msgTopicBusStateTest_t *message)
{
    uint32_t i;

    if (message == NULL) {
        return false;
    }

    if (message->publisher_tag_ != APP_TOPIC_BUS_TEST_STATE_TAG ||
        message->payload_pattern_ != appTopicBusTestBuildPattern(message->publish_seq_, APP_TOPIC_BUS_TEST_STATE_TAG)) {
        return false;
    }

    for (i = 0U; i < (sizeof(message->payload_words_) / sizeof(message->payload_words_[0])); i++) {
        if (message->payload_words_[i] != appTopicBusTestBuildStateWord(message->publish_seq_,
                                                                        APP_TOPIC_BUS_TEST_STATE_TAG,
                                                                        i)) {
            return false;
        }
    }

    return true;
}

static void appTopicBusTestTryMarkSuiteSuccess(void)
{
    taskENTER_CRITICAL();
    if (g_app_topic_bus_test_stats_.suite_done_ == 0U &&
        g_app_topic_bus_test_stats_.cmd_fast_passed_ != 0U &&
        g_app_topic_bus_test_stats_.cmd_slow_passed_ != 0U &&
        g_app_topic_bus_test_stats_.state_overwrite_passed_ != 0U) {
        g_app_topic_bus_test_stats_.suite_done_ = 1U;
        g_app_topic_bus_test_stats_.suite_passed_ = 1U;
        g_app_topic_bus_test_stats_.failure_code_ = (uint32_t)APP_TOPIC_BUS_TEST_FAILURE_NONE;
    }
    taskEXIT_CRITICAL();
}

static void appTopicBusTestCheckStop(void)
{
    if (g_app_topic_bus_test_stats_.suite_done_ != 0U) {
        vTaskSuspend(NULL);
    }
}

static moduleTopic_t *appTopicBusTestRequireTopic(appTopics_e topic_id)
{
    moduleTopic_t *topic = appTopicsGet(topic_id);

    if (topic == NULL) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_TOPIC_NOT_FOUND);
    }

    return topic;
}

static void appTopicBusTestValidateCmdMessage(const msgTopicBusCmdTest_t *message,
                                              uint32_t publisher_tag,
                                              appTopicBusTestFailure_e failure_code)
{
    if (message == NULL) {
        appTopicBusTestLatchFailure(failure_code);
    }

    if (message->publisher_tag_ != publisher_tag ||
        message->payload_pattern_ != appTopicBusTestBuildPattern(message->publish_seq_, publisher_tag)) {
        appTopicBusTestLatchFailure(failure_code);
    }
}

bool appTopicBusTestGetStats(appTopicBusTestStats_t *stats_out)
{
    if (stats_out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *stats_out = g_app_topic_bus_test_stats_;
    taskEXIT_CRITICAL();

    return true;
}

void appTopicBusTestPublisherTaskEntry(void *argument)
{
    moduleTopic_t *cmd_topic;
    moduleTopic_t *state_topic;
    uint32_t cmd_publish_seq = 0U;
    uint32_t state_publish_seq = 0U;

    (void)argument;

    cmd_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_CMD);
    state_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_STATE);

    for (;;) {
        msgTopicBusStateTest_t state_message;

        appTopicBusTestCheckStop();

        appTopicBusTestBuildStateMessage(&state_message,
                                         state_publish_seq,
                                         xTaskGetTickCount());

        if (moduleTopicBusPublish(state_topic, &state_message) == true) {
            g_app_topic_bus_test_stats_.state_publish_count_++;
        }
        state_publish_seq++;

        if ((state_publish_seq % (APP_TOPIC_BUS_TEST_CMD_PERIOD_MS / APP_TOPIC_BUS_TEST_STATE_PERIOD_MS)) == 0U) {
            msgTopicBusCmdTest_t cmd_message = {
                .publish_seq_ = cmd_publish_seq,
                .payload_pattern_ = appTopicBusTestBuildPattern(cmd_publish_seq, APP_TOPIC_BUS_TEST_CMD_TAG),
                .publish_tick_ = (uint32_t)xTaskGetTickCount(),
                .publisher_tag_ = APP_TOPIC_BUS_TEST_CMD_TAG,
            };

            if (moduleTopicBusPublish(cmd_topic, &cmd_message) == true) {
                g_app_topic_bus_test_stats_.cmd_publish_count_++;
            }
            cmd_publish_seq++;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_STATE_PERIOD_MS));
    }
}

void appTopicBusTestCommandFastSubscriberTaskEntry(void *argument)
{
    moduleTopic_t *cmd_topic;
    SemaphoreHandle_t wait_sem_handle;

    (void)argument;

    cmd_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_CMD);

    wait_sem_handle = xSemaphoreCreateBinaryStatic(&topic_bus_test_cmd_fast_sem_struct_);
    if (wait_sem_handle == NULL) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SEMAPHORE_CREATE);
    }

    topic_bus_test_cmd_fast_subscriber_.wait_handle_.wait_sem_ = wait_sem_handle;

    if (moduleTopicBusSubscribe(cmd_topic, &topic_bus_test_cmd_fast_subscriber_) == false) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SUBSCRIBE);
    }

    for (;;) {
        msgTopicBusCmdTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();

        if (moduleTopicBusWait(&topic_bus_test_cmd_fast_subscriber_,
                               pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS)) == false) {
            g_app_topic_bus_test_stats_.cmd_fast_wait_timeout_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_WAIT_TIMEOUT);
        }

        g_app_topic_bus_test_stats_.cmd_fast_wait_success_count_++;

        if (moduleTopicBusCopy(&topic_bus_test_cmd_fast_subscriber_, &message) == false) {
            g_app_topic_bus_test_stats_.cmd_fast_false_wakeup_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_FALSE_WAKEUP);
        }

        g_app_topic_bus_test_stats_.cmd_fast_copy_count_++;
        appTopicBusTestValidateCmdMessage(&message,
                                          APP_TOPIC_BUS_TEST_CMD_TAG,
                                          APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_PAYLOAD);

        if (g_app_topic_bus_test_stats_.cmd_fast_copy_count_ > 1U) {
            if (message.publish_seq_ <= g_app_topic_bus_test_stats_.cmd_fast_last_seq_) {
                g_app_topic_bus_test_stats_.cmd_fast_non_monotonic_count_++;
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_NON_MONOTONIC);
            }

            if (message.publish_seq_ > (g_app_topic_bus_test_stats_.cmd_fast_last_seq_ + 1U)) {
                seq_gap = message.publish_seq_ - g_app_topic_bus_test_stats_.cmd_fast_last_seq_ - 1U;
                g_app_topic_bus_test_stats_.cmd_fast_gap_count_ += seq_gap;
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_CMD_FAST_GAP);
            }
        }

        g_app_topic_bus_test_stats_.cmd_fast_last_seq_ = message.publish_seq_;

        if (g_app_topic_bus_test_stats_.cmd_fast_copy_count_ >= APP_TOPIC_BUS_TEST_CMD_FAST_TARGET) {
            g_app_topic_bus_test_stats_.cmd_fast_passed_ = 1U;
            appTopicBusTestTryMarkSuiteSuccess();
            vTaskSuspend(NULL);
        }
    }
}

void appTopicBusTestCommandSlowSubscriberTaskEntry(void *argument)
{
    moduleTopic_t *cmd_topic;

    (void)argument;

    cmd_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_CMD);

    if (moduleTopicBusSubscribe(cmd_topic, &topic_bus_test_cmd_slow_subscriber_) == false) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SUBSCRIBE);
    }

    for (;;) {
        msgTopicBusCmdTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();
        vTaskDelay(pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_CMD_SLOW_POLL_PERIOD_MS));
        g_app_topic_bus_test_stats_.cmd_slow_poll_count_++;

        if (moduleTopicBusUpdated(&topic_bus_test_cmd_slow_subscriber_) == false) {
            continue;
        }

        if (moduleTopicBusCopy(&topic_bus_test_cmd_slow_subscriber_, &message) == false) {
            continue;
        }

        g_app_topic_bus_test_stats_.cmd_slow_copy_count_++;
        appTopicBusTestValidateCmdMessage(&message,
                                          APP_TOPIC_BUS_TEST_CMD_TAG,
                                          APP_TOPIC_BUS_TEST_FAILURE_CMD_SLOW_PAYLOAD);

        if (g_app_topic_bus_test_stats_.cmd_slow_copy_count_ > 1U) {
            if (message.publish_seq_ <= g_app_topic_bus_test_stats_.cmd_slow_last_seq_) {
                g_app_topic_bus_test_stats_.cmd_slow_non_monotonic_count_++;
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_CMD_SLOW_NON_MONOTONIC);
            }

            if (message.publish_seq_ > (g_app_topic_bus_test_stats_.cmd_slow_last_seq_ + 1U)) {
                seq_gap = message.publish_seq_ - g_app_topic_bus_test_stats_.cmd_slow_last_seq_ - 1U;
                g_app_topic_bus_test_stats_.cmd_slow_gap_count_ += seq_gap;
            }
        }

        g_app_topic_bus_test_stats_.cmd_slow_last_seq_ = message.publish_seq_;
        g_app_topic_bus_test_stats_.cmd_slow_last_lost_count_ = topic_bus_test_cmd_slow_subscriber_.lost_count_;

        if ((g_app_topic_bus_test_stats_.cmd_slow_copy_count_ >= APP_TOPIC_BUS_TEST_CMD_SLOW_MIN_COPY_COUNT) &&
            (g_app_topic_bus_test_stats_.cmd_slow_gap_count_ > 0U) &&
            (g_app_topic_bus_test_stats_.cmd_slow_last_lost_count_ > 0U)) {
            g_app_topic_bus_test_stats_.cmd_slow_overwrite_observed_ = 1U;
            g_app_topic_bus_test_stats_.cmd_slow_passed_ = 1U;
            appTopicBusTestTryMarkSuiteSuccess();
            vTaskSuspend(NULL);
        }
    }
}

void appTopicBusTestStateSubscriberTaskEntry(void *argument)
{
    moduleTopic_t *state_topic;
    SemaphoreHandle_t wait_sem_handle;
    uint32_t lost_before_lag = 0U;

    (void)argument;

    state_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_STATE);
    g_app_topic_bus_test_stats_.state_phase_ = (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_INIT;

    wait_sem_handle = xSemaphoreCreateBinaryStatic(&topic_bus_test_state_sem_struct_);
    if (wait_sem_handle == NULL) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SEMAPHORE_CREATE);
    }

    topic_bus_test_state_subscriber_.wait_handle_.wait_sem_ = wait_sem_handle;

    if (moduleTopicBusSubscribe(state_topic, &topic_bus_test_state_subscriber_) == false) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SUBSCRIBE);
    }

    g_app_topic_bus_test_stats_.state_phase_ = (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_NO_LOSS;

    for (;;) {
        msgTopicBusStateTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();

        if (moduleTopicBusWait(&topic_bus_test_state_subscriber_,
                               pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS)) == false) {
            g_app_topic_bus_test_stats_.state_wait_timeout_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_WAIT_TIMEOUT);
        }

        g_app_topic_bus_test_stats_.state_wait_success_count_++;

        if (moduleTopicBusCopy(&topic_bus_test_state_subscriber_, &message) == false) {
            g_app_topic_bus_test_stats_.state_false_wakeup_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_FALSE_WAKEUP);
        }

        g_app_topic_bus_test_stats_.state_copy_count_++;
        if (appTopicBusTestValidateStateMessage(&message) == false) {
            g_app_topic_bus_test_stats_.state_payload_error_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_PAYLOAD);
        }

        if (g_app_topic_bus_test_stats_.state_copy_count_ > 1U) {
            if (message.publish_seq_ <= g_app_topic_bus_test_stats_.state_last_seq_) {
                g_app_topic_bus_test_stats_.state_non_monotonic_count_++;
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_NON_MONOTONIC);
            }

            if (message.publish_seq_ > (g_app_topic_bus_test_stats_.state_last_seq_ + 1U)) {
                seq_gap = message.publish_seq_ - g_app_topic_bus_test_stats_.state_last_seq_ - 1U;
                g_app_topic_bus_test_stats_.state_total_gap_count_ += seq_gap;
            }
        }

        g_app_topic_bus_test_stats_.state_last_seq_ = message.publish_seq_;
        g_app_topic_bus_test_stats_.state_last_lost_count_ = topic_bus_test_state_subscriber_.lost_count_;

        if (g_app_topic_bus_test_stats_.state_phase_ == (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_NO_LOSS) {
            g_app_topic_bus_test_stats_.state_phase1_copy_count_++;

            if (seq_gap != 0U) {
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_PHASE1_GAP);
            }

            if (g_app_topic_bus_test_stats_.state_phase1_copy_count_ >= APP_TOPIC_BUS_TEST_STATE_PHASE1_TARGET) {
                g_app_topic_bus_test_stats_.state_phase1_passed_ = 1U;
                g_app_topic_bus_test_stats_.state_phase_ = (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_FORCE_OVERWRITE;
                lost_before_lag = topic_bus_test_state_subscriber_.lost_count_;
                g_app_topic_bus_test_stats_.state_forced_lag_done_ = 1U;
                vTaskDelay(pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_STATE_FORCED_LAG_MS));
            }

            continue;
        }

        if (g_app_topic_bus_test_stats_.state_phase_ == (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_FORCE_OVERWRITE) {
            if (topic_bus_test_state_subscriber_.lost_count_ >= lost_before_lag) {
                g_app_topic_bus_test_stats_.state_overwrite_lost_delta_ =
                    topic_bus_test_state_subscriber_.lost_count_ - lost_before_lag;
            }
            g_app_topic_bus_test_stats_.state_overwrite_seq_gap_ = seq_gap;

            if (g_app_topic_bus_test_stats_.state_overwrite_lost_delta_ == 0U ||
                g_app_topic_bus_test_stats_.state_overwrite_seq_gap_ == 0U) {
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_NO_OVERWRITE);
            }

            g_app_topic_bus_test_stats_.state_phase_ = (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_DONE;
            g_app_topic_bus_test_stats_.state_overwrite_passed_ = 1U;
            appTopicBusTestTryMarkSuiteSuccess();
            vTaskSuspend(NULL);
        }
    }
}
