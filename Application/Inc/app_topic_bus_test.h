#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct app_topic_bus_test_stats
{
    uint32_t suite_done_;
    uint32_t suite_passed_;
    uint32_t failure_code_;
    uint32_t cmd_fast_passed_;
    uint32_t cmd_slow_passed_;
    uint32_t state_overwrite_passed_;

    uint32_t cmd_publish_count_;
    uint32_t state_publish_count_;

    uint32_t cmd_fast_wait_success_count_;
    uint32_t cmd_fast_wait_timeout_count_;
    uint32_t cmd_fast_copy_count_;
    uint32_t cmd_fast_false_wakeup_count_;
    uint32_t cmd_fast_payload_error_count_;
    uint32_t cmd_fast_non_monotonic_count_;
    uint32_t cmd_fast_gap_count_;
    uint32_t cmd_fast_last_seq_;

    uint32_t cmd_slow_poll_count_;
    uint32_t cmd_slow_copy_count_;
    uint32_t cmd_slow_payload_error_count_;
    uint32_t cmd_slow_non_monotonic_count_;
    uint32_t cmd_slow_gap_count_;
    uint32_t cmd_slow_last_seq_;
    uint32_t cmd_slow_last_lost_count_;
    uint32_t cmd_slow_overwrite_observed_;

    uint32_t state_phase_;
    uint32_t state_phase1_copy_count_;
    uint32_t state_phase1_passed_;
    uint32_t state_forced_lag_done_;
    uint32_t state_wait_success_count_;
    uint32_t state_wait_timeout_count_;
    uint32_t state_copy_count_;
    uint32_t state_false_wakeup_count_;
    uint32_t state_payload_error_count_;
    uint32_t state_non_monotonic_count_;
    uint32_t state_total_gap_count_;
    uint32_t state_last_seq_;
    uint32_t state_last_lost_count_;
    uint32_t state_overwrite_seq_gap_;
    uint32_t state_overwrite_lost_delta_;
} appTopicBusTestStats_t;

extern volatile appTopicBusTestStats_t g_app_topic_bus_test_stats_;

void appTopicBusTestPublisherTaskEntry(void *argument);
void appTopicBusTestCommandFastSubscriberTaskEntry(void *argument);
void appTopicBusTestCommandSlowSubscriberTaskEntry(void *argument);
void appTopicBusTestStateSubscriberTaskEntry(void *argument);
bool appTopicBusTestGetStats(appTopicBusTestStats_t *stats_out);
