#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "portmacro.h"
#include "stream_buffer.h"
#include "task.h"
#include "semphr.h"

#include "bsp_uart.h"

typedef enum
{
    MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER  = 0,
    MODULE_SERIAL_STREAM_BACKEND_ZERO_COPY,
} moduleSerialStreamBackend_e;

typedef enum
{
    MODULE_SERIAL_STREAM_NOTIFY_NONE = 0,
    MODULE_SERIAL_STREAM_NOTIFY_TASK,
    MODULE_SERIAL_STREAM_NOTIFY_SEMAPHORE,
} moduleSerialStreamNotifyBackend_e;

typedef union
{
    // StreamBuffer 模式使用
    StreamBufferHandle_t stream_buffer_handle_;

    // Zero copy 模式使用，由 notify_backend_ 决定解释成哪个 handle
    TaskHandle_t task_notify_handle_;
    SemaphoreHandle_t semphr_notify_handle_;
} moduleSerialStreamBackendHandle_u;

typedef struct module_serial_stream_config
{
    moduleSerialStreamBackend_e backend_;

    bspUARTInstance_t *uart_instance_;

    moduleSerialStreamBackendHandle_u backend_handle_;

    // 仅 Zero copy 模式使用；StreamBuffer 模式应保持 MODULE_SERIAL_STREAM_NOTIFY_NONE
    moduleSerialStreamNotifyBackend_e notify_backend_;
} moduleSerialStreamConfig_t;

typedef struct module_serial_stream
{
    moduleSerialStreamBackend_e backend_;

    bspUARTInstance_t *uart_instance_;

    moduleSerialStreamBackendHandle_u backend_handle_;
    moduleSerialStreamNotifyBackend_e notify_backend_;

    uint8_t *rx_dma_buffer_ptr_;
    uint16_t rx_dma_buffer_size_;

    volatile uint32_t rx_write_bytes_; // RX DMA 已写入字节总数
    uint32_t rx_read_bytes_;           // serial stream 已读取的字节总数，包含丢失后跳过的部分

    volatile uint32_t rx_total_bytes_;  // UART DMA 接收到并交给 serial stream 的总字节数
    volatile uint32_t rx_accept_bytes_; // 上层通过 moduleSerialStreamRead() 实际读出的总字节数
    volatile uint32_t rx_drop_count_;   // 丢弃次数
    volatile uint32_t rx_drop_bytes_;   // 丢弃字节数
} moduleSerialStream_t;

bool moduleSerialStreamInit(moduleSerialStream_t *instance, const moduleSerialStreamConfig_t *config);

// 从 serial stream 中读出数据，返回成功读出字节数；请注意 Zero copy 下 timeout_tick 无效，阻塞等待交给上层处理
uint32_t moduleSerialStreamRead(moduleSerialStream_t *instance,
                                uint8_t *data_out,
                                uint32_t max_length,
                                TickType_t timeout_tick);
