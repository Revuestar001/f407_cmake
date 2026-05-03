#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct app_uart6_serial_stream_rx_test_stats
{
    uint32_t init_ok_;
    uint32_t init_retry_count_;
    uint32_t init_fail_reason_;
    uint32_t backend_;
    uint32_t notify_backend_;

    uint32_t wait_timeout_count_;
    uint32_t wait_wakeup_count_;
    uint32_t task_notify_take_count_;
    uint32_t semaphore_take_count_;

    uint32_t read_count_;
    uint32_t read_total_bytes_;
    uint32_t read_zero_length_count_;
    uint32_t line_count_;
    uint32_t line_overflow_count_;
    uint32_t line_buffered_bytes_;

    uint32_t last_chunk_bytes_;
    uint32_t max_chunk_bytes_;
    uint32_t last_read_tick_ms_;
    uint32_t last_inter_read_gap_ms_;
    uint32_t max_inter_read_gap_ms_;

    uint32_t serial_total_bytes_;
    uint32_t serial_accept_bytes_;
    uint32_t serial_drop_count_;
    uint32_t serial_drop_bytes_;

    uint32_t stack_hwm_words_;
    uint32_t stack_hwm_min_words_;

    uint8_t last_chunk_[64];
    uint32_t last_chunk_length_;
    uint8_t last_line_[64];
    uint32_t last_line_length_;
} appUART6SerialStreamRxTestStats_t;

extern volatile appUART6SerialStreamRxTestStats_t g_app_uart6_serial_stream_rx_test_stats_;

void appUART6SerialStreamRxTestTaskEntry(void *argument);
bool appUART6SerialStreamRxTestGetStats(appUART6SerialStreamRxTestStats_t *stats_out);
