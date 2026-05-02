#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_topic_bus_test.h"
#include "app_topics.h"
#include "msg_topic_bus_test.h"
#include "topic_bus.h"
#include "topic_bus_private.h"

#define APP_TOPIC_BUS_TEST_DURATION_MS 300000U
#define APP_TOPIC_BUS_TEST_CMD_PERIOD_MS 20U
#define APP_TOPIC_BUS_TEST_STATE_PERIOD_MS 1U
#define APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS 200U
#define APP_TOPIC_BUS_TEST_CMD_SLOW_POLL_PERIOD_MS 100U
#define APP_TOPIC_BUS_TEST_STATE_PHASE1_TARGET 10000U
#define APP_TOPIC_BUS_TEST_STATE_FORCED_LAG_MS 20U
#define APP_TOPIC_BUS_TEST_PAYLOAD_PATTERN_BASE 0xA55A5AA5UL
#define APP_TOPIC_BUS_TEST_CMD_TAG 0x434D4421UL
#define APP_TOPIC_BUS_TEST_STATE_TAG 0x53544154UL
#define APP_TOPIC_BUS_TEST_STATE_AUX_TAG 0x41555832UL

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
    APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_WAIT_TIMEOUT = 18U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_FALSE_WAKEUP = 19U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_PAYLOAD = 20U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_NON_MONOTONIC = 21U,
    APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_GAP = 22U,
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
static StaticSemaphore_t topic_bus_test_state_aux_sem_struct_;
static TaskHandle_t topic_bus_test_publisher_task_handle_ = NULL;
static TaskHandle_t topic_bus_test_cmd_fast_task_handle_ = NULL;
static TaskHandle_t topic_bus_test_state_task_handle_ = NULL;
static TaskHandle_t topic_bus_test_state_aux_task_handle_ = NULL;

static moduleTopicSubscription_t topic_bus_test_cmd_fast_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_SEMAPHORE,
};
static moduleTopicSubscription_t topic_bus_test_cmd_slow_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_NONE,
};
static moduleTopicSubscription_t topic_bus_test_state_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_SEMAPHORE,
};
static moduleTopicSubscription_t topic_bus_test_state_aux_subscriber_ = {
    .wait_backend_ = MODULE_TOPIC_WAIT_SEMAPHORE,
};

volatile appTopicBusTestStats_t g_app_topic_bus_test_stats_ = {0};

static bool appTopicBusTestDurationReached(TickType_t start_tick)
{
    return (xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_DURATION_MS);
}

static uint32_t appTopicBusTestTickToMs(TickType_t tick)
{
    return (uint32_t)(tick * portTICK_PERIOD_MS);
}

static uint32_t appTopicBusTestGetTopicPublishSeq(const moduleTopic_t *topic)
{
    if (topic == NULL) {
        return 0U;
    }

    return topic->publish_seq_num_;
}

static uint32_t appTopicBusTestGetTaskStateSnapshot(TaskHandle_t task_handle)
{
    if (task_handle == NULL) {
        return (uint32_t)eInvalid;
    }

    return (uint32_t)eTaskGetState(task_handle);
}

static uint32_t appTopicBusTestGetTaskStackHWMWords(TaskHandle_t task_handle)
{
    if (task_handle == NULL) {
        return 0U;
    }

    return (uint32_t)uxTaskGetStackHighWaterMark(task_handle);
}

static void appTopicBusTestUpdateMinWaterMark(uint32_t *min_hwm_words, uint32_t current_hwm_words)
{
    if (min_hwm_words == NULL) {
        return;
    }

    if (*min_hwm_words == 0U || current_hwm_words < *min_hwm_words) {
        *min_hwm_words = current_hwm_words;
    }
}

static void appTopicBusTestUpdatePublisherTaskDiagnostics(void)
{
    uint32_t current_hwm_words = appTopicBusTestGetTaskStackHWMWords(topic_bus_test_publisher_task_handle_);

    g_app_topic_bus_test_stats_.publisher_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_publisher_task_handle_);
    g_app_topic_bus_test_stats_.publisher_stack_hwm_words_ = current_hwm_words;
    appTopicBusTestUpdateMinWaterMark(&g_app_topic_bus_test_stats_.publisher_stack_hwm_min_words_,
                                      current_hwm_words);
}

static void appTopicBusTestUpdateCmdFastTaskDiagnostics(void)
{
    uint32_t current_hwm_words = appTopicBusTestGetTaskStackHWMWords(topic_bus_test_cmd_fast_task_handle_);

    g_app_topic_bus_test_stats_.cmd_fast_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_cmd_fast_task_handle_);
    g_app_topic_bus_test_stats_.cmd_fast_stack_hwm_words_ = current_hwm_words;
    appTopicBusTestUpdateMinWaterMark(&g_app_topic_bus_test_stats_.cmd_fast_stack_hwm_min_words_,
                                      current_hwm_words);
}

static void appTopicBusTestUpdateStateTaskDiagnostics(void)
{
    uint32_t current_hwm_words = appTopicBusTestGetTaskStackHWMWords(topic_bus_test_state_task_handle_);

    g_app_topic_bus_test_stats_.state_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_state_task_handle_);
    g_app_topic_bus_test_stats_.state_stack_hwm_words_ = current_hwm_words;
    appTopicBusTestUpdateMinWaterMark(&g_app_topic_bus_test_stats_.state_stack_hwm_min_words_,
                                      current_hwm_words);
}

static void appTopicBusTestUpdateStateAuxTaskDiagnostics(void)
{
    uint32_t current_hwm_words = appTopicBusTestGetTaskStackHWMWords(topic_bus_test_state_aux_task_handle_);

    g_app_topic_bus_test_stats_.state_aux_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_state_aux_task_handle_);
    g_app_topic_bus_test_stats_.state_aux_stack_hwm_words_ = current_hwm_words;
    appTopicBusTestUpdateMinWaterMark(&g_app_topic_bus_test_stats_.state_aux_stack_hwm_min_words_,
                                      current_hwm_words);
}

static void appTopicBusTestRefreshSuiteElapsedMs(TickType_t suite_start_tick)
{
    g_app_topic_bus_test_stats_.suite_elapsed_ms_ =
        appTopicBusTestTickToMs(xTaskGetTickCount() - suite_start_tick);
}

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
                                             TickType_t publish_tick,
                                             uint32_t publisher_tag)
{
    uint32_t i;

    if (message == NULL) {
        return;
    }

    message->publish_seq_ = publish_seq;
    message->payload_pattern_ = appTopicBusTestBuildPattern(publish_seq, publisher_tag);
    message->publish_tick_ = (uint32_t)publish_tick;
    message->publisher_tag_ = publisher_tag;

    for (i = 0U; i < (sizeof(message->payload_words_) / sizeof(message->payload_words_[0])); i++) {
        message->payload_words_[i] = appTopicBusTestBuildStateWord(publish_seq,
                                                                   publisher_tag,
                                                                   i);
    }
}

static bool appTopicBusTestValidateStateMessage(const msgTopicBusStateTest_t *message,
                                                uint32_t publisher_tag)
{
    uint32_t i;

    if (message == NULL) {
        return false;
    }

    if (message->publisher_tag_ != publisher_tag ||
        message->payload_pattern_ != appTopicBusTestBuildPattern(message->publish_seq_, publisher_tag)) {
        return false;
    }

    for (i = 0U; i < (sizeof(message->payload_words_) / sizeof(message->payload_words_[0])); i++) {
        if (message->payload_words_[i] != appTopicBusTestBuildStateWord(message->publish_seq_,
                                                                        publisher_tag,
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
        g_app_topic_bus_test_stats_.state_overwrite_passed_ != 0U &&
        g_app_topic_bus_test_stats_.state_aux_passed_ != 0U) {
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

static void appTopicBusTestCaptureCmdFastWaitContextBefore(const moduleTopic_t *cmd_topic)
{
    g_app_topic_bus_test_stats_.cmd_fast_wait_start_ms_ = appTopicBusTestTickToMs(xTaskGetTickCount());
    g_app_topic_bus_test_stats_.cmd_fast_wait_publish_seq_before_ = appTopicBusTestGetTopicPublishSeq(cmd_topic);
    g_app_topic_bus_test_stats_.cmd_fast_wait_next_seq_before_ = topic_bus_test_cmd_fast_subscriber_.next_subscribe_seq_num_;
    g_app_topic_bus_test_stats_.cmd_fast_wait_updated_before_ =
        moduleTopicBusUpdated(&topic_bus_test_cmd_fast_subscriber_) ? 1U : 0U;
}

static void appTopicBusTestCaptureCmdFastWaitTimeoutContext(const moduleTopic_t *cmd_topic)
{
    g_app_topic_bus_test_stats_.cmd_fast_wait_publish_seq_after_timeout_ = appTopicBusTestGetTopicPublishSeq(cmd_topic);
    g_app_topic_bus_test_stats_.cmd_fast_wait_next_seq_after_timeout_ = topic_bus_test_cmd_fast_subscriber_.next_subscribe_seq_num_;
    g_app_topic_bus_test_stats_.cmd_fast_wait_updated_after_timeout_ =
        moduleTopicBusUpdated(&topic_bus_test_cmd_fast_subscriber_) ? 1U : 0U;
    g_app_topic_bus_test_stats_.cmd_fast_wait_publisher_loop_count_snapshot_ =
        g_app_topic_bus_test_stats_.publisher_loop_count_;
    g_app_topic_bus_test_stats_.cmd_fast_wait_publisher_last_loop_tick_ms_snapshot_ =
        g_app_topic_bus_test_stats_.publisher_last_loop_tick_ms_;
    g_app_topic_bus_test_stats_.cmd_fast_wait_publisher_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_publisher_task_handle_);
    g_app_topic_bus_test_stats_.cmd_fast_wait_publisher_stack_hwm_words_snapshot_ =
        appTopicBusTestGetTaskStackHWMWords(topic_bus_test_publisher_task_handle_);
}

static void appTopicBusTestCaptureStateWaitContextBefore(const moduleTopic_t *state_topic)
{
    g_app_topic_bus_test_stats_.state_wait_start_ms_ = appTopicBusTestTickToMs(xTaskGetTickCount());
    g_app_topic_bus_test_stats_.state_wait_publish_seq_before_ = appTopicBusTestGetTopicPublishSeq(state_topic);
    g_app_topic_bus_test_stats_.state_wait_next_seq_before_ = topic_bus_test_state_subscriber_.next_subscribe_seq_num_;
    g_app_topic_bus_test_stats_.state_wait_lost_count_before_ = topic_bus_test_state_subscriber_.lost_count_;
    g_app_topic_bus_test_stats_.state_wait_updated_before_ =
        moduleTopicBusUpdated(&topic_bus_test_state_subscriber_) ? 1U : 0U;
}

static void appTopicBusTestCaptureStateWaitTimeoutContext(const moduleTopic_t *state_topic)
{
    g_app_topic_bus_test_stats_.state_wait_publish_seq_after_timeout_ = appTopicBusTestGetTopicPublishSeq(state_topic);
    g_app_topic_bus_test_stats_.state_wait_next_seq_after_timeout_ = topic_bus_test_state_subscriber_.next_subscribe_seq_num_;
    g_app_topic_bus_test_stats_.state_wait_lost_count_after_timeout_ = topic_bus_test_state_subscriber_.lost_count_;
    g_app_topic_bus_test_stats_.state_wait_updated_after_timeout_ =
        moduleTopicBusUpdated(&topic_bus_test_state_subscriber_) ? 1U : 0U;
    g_app_topic_bus_test_stats_.state_wait_timeout_phase_snapshot_ = g_app_topic_bus_test_stats_.state_phase_;
    g_app_topic_bus_test_stats_.state_wait_timeout_suite_elapsed_ms_snapshot_ =
        g_app_topic_bus_test_stats_.suite_elapsed_ms_;
    g_app_topic_bus_test_stats_.state_wait_publisher_loop_count_snapshot_ =
        g_app_topic_bus_test_stats_.publisher_loop_count_;
    g_app_topic_bus_test_stats_.state_wait_publisher_last_loop_tick_ms_snapshot_ =
        g_app_topic_bus_test_stats_.publisher_last_loop_tick_ms_;
    g_app_topic_bus_test_stats_.state_wait_publisher_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_publisher_task_handle_);
    g_app_topic_bus_test_stats_.state_wait_publisher_stack_hwm_words_snapshot_ =
        appTopicBusTestGetTaskStackHWMWords(topic_bus_test_publisher_task_handle_);
    g_app_topic_bus_test_stats_.state_wait_self_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_state_task_handle_);
    g_app_topic_bus_test_stats_.state_wait_self_stack_hwm_words_snapshot_ =
        appTopicBusTestGetTaskStackHWMWords(topic_bus_test_state_task_handle_);
    g_app_topic_bus_test_stats_.state_wait_state_aux_publish_count_snapshot_ =
        g_app_topic_bus_test_stats_.state_aux_publish_count_;
    g_app_topic_bus_test_stats_.state_wait_state_aux_copy_count_snapshot_ =
        g_app_topic_bus_test_stats_.state_aux_copy_count_;
    g_app_topic_bus_test_stats_.state_wait_state_aux_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_state_aux_task_handle_);
    g_app_topic_bus_test_stats_.state_wait_state_aux_stack_hwm_words_snapshot_ =
        appTopicBusTestGetTaskStackHWMWords(topic_bus_test_state_aux_task_handle_);
}

static void appTopicBusTestCaptureStateAuxWaitContextBefore(const moduleTopic_t *state_aux_topic)
{
    g_app_topic_bus_test_stats_.state_aux_wait_start_ms_ = appTopicBusTestTickToMs(xTaskGetTickCount());
    g_app_topic_bus_test_stats_.state_aux_wait_publish_seq_before_ = appTopicBusTestGetTopicPublishSeq(state_aux_topic);
    g_app_topic_bus_test_stats_.state_aux_wait_next_seq_before_ = topic_bus_test_state_aux_subscriber_.next_subscribe_seq_num_;
    g_app_topic_bus_test_stats_.state_aux_wait_updated_before_ =
        moduleTopicBusUpdated(&topic_bus_test_state_aux_subscriber_) ? 1U : 0U;
}

static void appTopicBusTestCaptureStateAuxWaitTimeoutContext(const moduleTopic_t *state_aux_topic)
{
    g_app_topic_bus_test_stats_.state_aux_wait_publish_seq_after_timeout_ = appTopicBusTestGetTopicPublishSeq(state_aux_topic);
    g_app_topic_bus_test_stats_.state_aux_wait_next_seq_after_timeout_ = topic_bus_test_state_aux_subscriber_.next_subscribe_seq_num_;
    g_app_topic_bus_test_stats_.state_aux_wait_updated_after_timeout_ =
        moduleTopicBusUpdated(&topic_bus_test_state_aux_subscriber_) ? 1U : 0U;
    g_app_topic_bus_test_stats_.state_aux_wait_publisher_loop_count_snapshot_ =
        g_app_topic_bus_test_stats_.publisher_loop_count_;
    g_app_topic_bus_test_stats_.state_aux_wait_publisher_last_loop_tick_ms_snapshot_ =
        g_app_topic_bus_test_stats_.publisher_last_loop_tick_ms_;
    g_app_topic_bus_test_stats_.state_aux_wait_publisher_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_publisher_task_handle_);
    g_app_topic_bus_test_stats_.state_aux_wait_publisher_stack_hwm_words_snapshot_ =
        appTopicBusTestGetTaskStackHWMWords(topic_bus_test_publisher_task_handle_);
    g_app_topic_bus_test_stats_.state_aux_wait_self_task_state_snapshot_ =
        appTopicBusTestGetTaskStateSnapshot(topic_bus_test_state_aux_task_handle_);
    g_app_topic_bus_test_stats_.state_aux_wait_self_stack_hwm_words_snapshot_ =
        appTopicBusTestGetTaskStackHWMWords(topic_bus_test_state_aux_task_handle_);
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
    moduleTopic_t *state_aux_topic;
    TickType_t suite_start_tick;
    uint32_t cmd_publish_seq = 0U;
    uint32_t state_publish_seq = 0U;
    uint32_t state_aux_publish_seq = 0U;

    (void)argument;

    cmd_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_CMD);
    state_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_STATE);
    state_aux_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_STATE_AUX);
    topic_bus_test_publisher_task_handle_ = xTaskGetCurrentTaskHandle();

    g_app_topic_bus_test_stats_.suite_target_ms_ = APP_TOPIC_BUS_TEST_DURATION_MS;
    suite_start_tick = xTaskGetTickCount();

    for (;;) {
        msgTopicBusStateTest_t state_message;
        msgTopicBusStateTest_t state_aux_message;
        TickType_t publish_tick = xTaskGetTickCount();

        appTopicBusTestCheckStop();
        appTopicBusTestRefreshSuiteElapsedMs(suite_start_tick);
        g_app_topic_bus_test_stats_.publisher_loop_count_++;
        g_app_topic_bus_test_stats_.publisher_last_loop_tick_ms_ = appTopicBusTestTickToMs(publish_tick);
        appTopicBusTestUpdatePublisherTaskDiagnostics();

        appTopicBusTestBuildStateMessage(&state_message,
                                         state_publish_seq,
                                         publish_tick,
                                         APP_TOPIC_BUS_TEST_STATE_TAG);
        if (moduleTopicBusPublish(state_topic, &state_message) == true) {
            g_app_topic_bus_test_stats_.state_publish_count_++;
        }
        state_publish_seq++;

        appTopicBusTestBuildStateMessage(&state_aux_message,
                                         state_aux_publish_seq,
                                         publish_tick,
                                         APP_TOPIC_BUS_TEST_STATE_AUX_TAG);
        if (moduleTopicBusPublish(state_aux_topic, &state_aux_message) == true) {
            g_app_topic_bus_test_stats_.state_aux_publish_count_++;
        }
        state_aux_publish_seq++;

        if ((state_publish_seq % (APP_TOPIC_BUS_TEST_CMD_PERIOD_MS / APP_TOPIC_BUS_TEST_STATE_PERIOD_MS)) == 0U) {
            msgTopicBusCmdTest_t cmd_message = {
                .publish_seq_ = cmd_publish_seq,
                .payload_pattern_ = appTopicBusTestBuildPattern(cmd_publish_seq, APP_TOPIC_BUS_TEST_CMD_TAG),
                .publish_tick_ = (uint32_t)publish_tick,
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
    TickType_t task_start_tick;

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

    topic_bus_test_cmd_fast_task_handle_ = xTaskGetCurrentTaskHandle();
    task_start_tick = xTaskGetTickCount();

    for (;;) {
        msgTopicBusCmdTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();
        appTopicBusTestUpdateCmdFastTaskDiagnostics();
        appTopicBusTestCaptureCmdFastWaitContextBefore(cmd_topic);

        if (moduleTopicBusWait(&topic_bus_test_cmd_fast_subscriber_,
                               pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS)) == false) {
            g_app_topic_bus_test_stats_.cmd_fast_wait_timeout_count_++;
            appTopicBusTestCaptureCmdFastWaitTimeoutContext(cmd_topic);
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

        if (appTopicBusTestDurationReached(task_start_tick) == true) {
            g_app_topic_bus_test_stats_.cmd_fast_passed_ = 1U;
            appTopicBusTestTryMarkSuiteSuccess();
            vTaskSuspend(NULL);
        }
    }
}

void appTopicBusTestCommandSlowSubscriberTaskEntry(void *argument)
{
    moduleTopic_t *cmd_topic;
    TickType_t task_start_tick;

    (void)argument;

    cmd_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_CMD);

    if (moduleTopicBusSubscribe(cmd_topic, &topic_bus_test_cmd_slow_subscriber_) == false) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SUBSCRIBE);
    }

    task_start_tick = xTaskGetTickCount();

    for (;;) {
        msgTopicBusCmdTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();
        vTaskDelay(pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_CMD_SLOW_POLL_PERIOD_MS));
        g_app_topic_bus_test_stats_.cmd_slow_poll_count_++;

        if (moduleTopicBusUpdated(&topic_bus_test_cmd_slow_subscriber_) == true) {
            if (moduleTopicBusCopy(&topic_bus_test_cmd_slow_subscriber_, &message) == true) {
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

                if (g_app_topic_bus_test_stats_.cmd_slow_gap_count_ > 0U &&
                    g_app_topic_bus_test_stats_.cmd_slow_last_lost_count_ > 0U) {
                    g_app_topic_bus_test_stats_.cmd_slow_overwrite_observed_ = 1U;
                }
            }
        }

        if (appTopicBusTestDurationReached(task_start_tick) == true) {
            if (g_app_topic_bus_test_stats_.cmd_slow_overwrite_observed_ == 0U) {
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_CMD_SLOW_NO_OVERWRITE);
            }

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
    TickType_t task_start_tick;
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

    topic_bus_test_state_task_handle_ = xTaskGetCurrentTaskHandle();
    task_start_tick = xTaskGetTickCount();
    g_app_topic_bus_test_stats_.state_phase_ = (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_NO_LOSS;

    for (;;) {
        msgTopicBusStateTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();
        appTopicBusTestUpdateStateTaskDiagnostics();
        appTopicBusTestCaptureStateWaitContextBefore(state_topic);

        if (moduleTopicBusWait(&topic_bus_test_state_subscriber_,
                               pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS)) == false) {
            g_app_topic_bus_test_stats_.state_wait_timeout_count_++;
            appTopicBusTestCaptureStateWaitTimeoutContext(state_topic);
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_WAIT_TIMEOUT);
        }

        g_app_topic_bus_test_stats_.state_wait_success_count_++;

        if (moduleTopicBusCopy(&topic_bus_test_state_subscriber_, &message) == false) {
            g_app_topic_bus_test_stats_.state_false_wakeup_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_FALSE_WAKEUP);
        }

        g_app_topic_bus_test_stats_.state_copy_count_++;
        if (appTopicBusTestValidateStateMessage(&message, APP_TOPIC_BUS_TEST_STATE_TAG) == false) {
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
                continue;
            }
        } else if (g_app_topic_bus_test_stats_.state_phase_ == (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_FORCE_OVERWRITE) {
            if (topic_bus_test_state_subscriber_.lost_count_ >= lost_before_lag) {
                g_app_topic_bus_test_stats_.state_overwrite_lost_delta_ =
                    topic_bus_test_state_subscriber_.lost_count_ - lost_before_lag;
            }
            g_app_topic_bus_test_stats_.state_overwrite_seq_gap_ = seq_gap;

            if (g_app_topic_bus_test_stats_.state_overwrite_lost_delta_ == 0U ||
                g_app_topic_bus_test_stats_.state_overwrite_seq_gap_ == 0U) {
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_NO_OVERWRITE);
            }

            g_app_topic_bus_test_stats_.state_overwrite_observed_ = 1U;
            g_app_topic_bus_test_stats_.state_phase_ = (uint32_t)APP_TOPIC_BUS_TEST_STATE_PHASE_DONE;
        }

        if (appTopicBusTestDurationReached(task_start_tick) == true) {
            if (g_app_topic_bus_test_stats_.state_overwrite_observed_ == 0U) {
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_NO_OVERWRITE);
            }

            g_app_topic_bus_test_stats_.state_overwrite_passed_ = 1U;
            appTopicBusTestTryMarkSuiteSuccess();
            vTaskSuspend(NULL);
        }
    }
}

void appTopicBusTestStateAuxSubscriberTaskEntry(void *argument)
{
    moduleTopic_t *state_aux_topic;
    SemaphoreHandle_t wait_sem_handle;
    TickType_t task_start_tick;

    (void)argument;

    state_aux_topic = appTopicBusTestRequireTopic(APP_TOPICS_TOPIC_BUS_TEST_STATE_AUX);

    wait_sem_handle = xSemaphoreCreateBinaryStatic(&topic_bus_test_state_aux_sem_struct_);
    if (wait_sem_handle == NULL) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SEMAPHORE_CREATE);
    }

    topic_bus_test_state_aux_subscriber_.wait_handle_.wait_sem_ = wait_sem_handle;

    if (moduleTopicBusSubscribe(state_aux_topic, &topic_bus_test_state_aux_subscriber_) == false) {
        appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_SUBSCRIBE);
    }

    topic_bus_test_state_aux_task_handle_ = xTaskGetCurrentTaskHandle();
    task_start_tick = xTaskGetTickCount();

    for (;;) {
        msgTopicBusStateTest_t message;
        uint32_t seq_gap = 0U;

        appTopicBusTestCheckStop();
        appTopicBusTestUpdateStateAuxTaskDiagnostics();
        appTopicBusTestCaptureStateAuxWaitContextBefore(state_aux_topic);

        if (moduleTopicBusWait(&topic_bus_test_state_aux_subscriber_,
                               pdMS_TO_TICKS(APP_TOPIC_BUS_TEST_WAIT_TIMEOUT_MS)) == false) {
            g_app_topic_bus_test_stats_.state_aux_wait_timeout_count_++;
            appTopicBusTestCaptureStateAuxWaitTimeoutContext(state_aux_topic);
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_WAIT_TIMEOUT);
        }

        g_app_topic_bus_test_stats_.state_aux_wait_success_count_++;

        if (moduleTopicBusCopy(&topic_bus_test_state_aux_subscriber_, &message) == false) {
            g_app_topic_bus_test_stats_.state_aux_false_wakeup_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_FALSE_WAKEUP);
        }

        g_app_topic_bus_test_stats_.state_aux_copy_count_++;
        if (appTopicBusTestValidateStateMessage(&message, APP_TOPIC_BUS_TEST_STATE_AUX_TAG) == false) {
            g_app_topic_bus_test_stats_.state_aux_payload_error_count_++;
            appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_PAYLOAD);
        }

        if (g_app_topic_bus_test_stats_.state_aux_copy_count_ > 1U) {
            if (message.publish_seq_ <= g_app_topic_bus_test_stats_.state_aux_last_seq_) {
                g_app_topic_bus_test_stats_.state_aux_non_monotonic_count_++;
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_NON_MONOTONIC);
            }

            if (message.publish_seq_ > (g_app_topic_bus_test_stats_.state_aux_last_seq_ + 1U)) {
                seq_gap = message.publish_seq_ - g_app_topic_bus_test_stats_.state_aux_last_seq_ - 1U;
                g_app_topic_bus_test_stats_.state_aux_gap_count_ += seq_gap;
                appTopicBusTestLatchFailure(APP_TOPIC_BUS_TEST_FAILURE_STATE_AUX_GAP);
            }
        }

        g_app_topic_bus_test_stats_.state_aux_last_seq_ = message.publish_seq_;

        if (appTopicBusTestDurationReached(task_start_tick) == true) {
            g_app_topic_bus_test_stats_.state_aux_passed_ = 1U;
            appTopicBusTestTryMarkSuiteSuccess();
            vTaskSuspend(NULL);
        }
    }
}
