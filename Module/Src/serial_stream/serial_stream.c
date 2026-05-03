#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_uart.h"
#include "serial_stream.h"

static bool isBackendValid(moduleSerialStreamBackend_e backend)
{
    switch (backend) {
        case MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER:
        case MODULE_SERIAL_STREAM_BACKEND_DMA_RING_BUFFER:
            return true;
        default:
            break;
    }

    return false;
}

static bool isNotifyBackendValid(moduleSerialStreamNotifyBackend_e notify_backend)
{
    switch (notify_backend) {
        case MODULE_SERIAL_STREAM_NOTIFY_NONE:
        case MODULE_SERIAL_STREAM_NOTIFY_TASK:
        case MODULE_SERIAL_STREAM_NOTIFY_SEMAPHORE:
            return true;
        default:
            break;
    }

    return false;
}

// backend_handle_ 是 union，必须结合 notify_backend_ 才能判断 DMA ring buffer 的接收通知句柄是否有效
static bool isDMARingBufferNotifyHandleValid(const moduleSerialStream_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    switch (instance->notify_backend_) {
        case MODULE_SERIAL_STREAM_NOTIFY_TASK:
            return instance->backend_handle_.task_notify_handle_ != NULL;

        case MODULE_SERIAL_STREAM_NOTIFY_SEMAPHORE:
            return instance->backend_handle_.semphr_notify_handle_ != NULL;

        default:
            break;
    }

    return false;
}

// backend_handle_ 是 union，必须结合 notify_backend_ 才能判断 DMA ring buffer 的配置是否有效
static bool isDMARingBufferNotifyConfigValid(const moduleSerialStreamConfig_t *config)
{
    if (config == NULL) {
        return false;
    }

    switch (config->notify_backend_) {
        case MODULE_SERIAL_STREAM_NOTIFY_TASK:
            return config->backend_handle_.task_notify_handle_ != NULL;

        case MODULE_SERIAL_STREAM_NOTIFY_SEMAPHORE:
            return config->backend_handle_.semphr_notify_handle_ != NULL;

        default:
            break;
    }

    return false;
}

static void notifyDMARingBufferReceiverFromISR(moduleSerialStream_t *instance,
                                               BaseType_t *xHigherPriorityTaskWoken)
{
    if (instance == NULL || xHigherPriorityTaskWoken == NULL) {
        return;
    }

    switch (instance->notify_backend_) {
        case MODULE_SERIAL_STREAM_NOTIFY_TASK:
            if (instance->backend_handle_.task_notify_handle_ == NULL) {
                return;
            }
            vTaskNotifyGiveFromISR(instance->backend_handle_.task_notify_handle_, xHigherPriorityTaskWoken);
            break;
        case MODULE_SERIAL_STREAM_NOTIFY_SEMAPHORE:
            if (instance->backend_handle_.semphr_notify_handle_ == NULL) {
                return;
            }
            xSemaphoreGiveFromISR(instance->backend_handle_.semphr_notify_handle_, xHigherPriorityTaskWoken);
            break;
        default:
            break;
    }
}

// StreamBuffer 模式，向 StreamBuffer 中写入 DMA 缓冲区的数据
static void sendSegmentToBufferFromISR(moduleSerialStream_t *instance,
                                       uint8_t *rx_buffer_ptr,
                                       uint16_t rx_buffer_size,
                                       uint16_t rx_data_start_index,
                                       uint16_t rx_data_end_pos,
                                       BaseType_t *xHigherPriorityTaskWoken)
{
    if (instance == NULL ||
        xHigherPriorityTaskWoken == NULL ||
        instance->backend_handle_.stream_buffer_handle_ == NULL ||
        rx_buffer_ptr == NULL ||
        rx_data_start_index >= rx_buffer_size ||
        rx_data_end_pos > rx_buffer_size ||
        rx_data_start_index >= rx_data_end_pos) {
        return;
    }

    size_t bytes_to_send;
    size_t bytes_sent;
    bytes_to_send = (size_t)(rx_data_end_pos - rx_data_start_index);
    bytes_sent = xStreamBufferSendFromISR(instance->backend_handle_.stream_buffer_handle_,
                                          &rx_buffer_ptr[rx_data_start_index],
                                          bytes_to_send,
                                          xHigherPriorityTaskWoken);
    instance->rx_total_bytes_ += (uint32_t)bytes_to_send;

    // 送入的字节数大于实际写入字节数，说明 StreamBuffer 容量不够，发生了丢字节
    if (bytes_sent < bytes_to_send) {
        instance->rx_drop_count_++;
        instance->rx_drop_bytes_ += (uint32_t)(bytes_to_send - bytes_sent);
    }
}

static uint32_t readDataDMARingBuffer(moduleSerialStream_t *instance,
                                      uint8_t *data_out,
                                      uint32_t max_length)
{
    if (instance == NULL || data_out == NULL || max_length == 0U) {
        return 0U;
    }

    if (instance->rx_dma_buffer_ptr_ == NULL || instance->rx_dma_buffer_size_ == 0U) {
        return 0U;
    }

    uint32_t write_bytes = instance->rx_write_bytes_;
    uint32_t read_bytes = instance->rx_read_bytes_;
    uint16_t dma_size = instance->rx_dma_buffer_size_;
    
    uint32_t available_bytes = write_bytes - read_bytes;

    if (available_bytes == 0U) {
        return 0U;
    }

    // 本次需要读取的字节数超出 DMA 缓冲区大小，说明 DMA 已经覆盖了未读取的数据
    if (available_bytes > dma_size) {
        uint32_t lost_bytes = available_bytes - dma_size;

        instance->rx_drop_count_++;
        instance->rx_drop_bytes_ += lost_bytes;

        // 移动到 DMA 缓冲区内当前还能保证未被覆盖的最旧字节开始读
        read_bytes = write_bytes - dma_size;
        instance->rx_read_bytes_ = read_bytes;
        available_bytes = dma_size;
    }

    uint32_t bytes_to_read = (available_bytes < max_length) ? available_bytes : max_length;

    for (uint32_t i = 0U; i < bytes_to_read; i++) {
        uint16_t read_index = (uint16_t)(instance->rx_read_bytes_ % dma_size);

        data_out[i] = instance->rx_dma_buffer_ptr_[read_index];
        instance->rx_read_bytes_++;
    }

    instance->rx_accept_bytes_ += bytes_to_read;

    return bytes_to_read;
}

// UART RX 回调，在中断上下文中由 bsp_uart 调用
static void uartRxCallbackFromISR(void *owner_ptr, const bspUARTRxEventContext_t *rx_context)
{
    if (owner_ptr == NULL ||
        rx_context == NULL ||
        rx_context->rx_buffer_ptr_ == NULL ||
        rx_context->rx_buffer_size_ == 0U ||
        rx_context->rx_data_len_ == 0U) {
        return;
    }

    moduleSerialStream_t *instance = (moduleSerialStream_t *)owner_ptr;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (instance->backend_) {
        case MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER: {
            uint16_t start = rx_context->rx_data_start_index_;
            uint16_t end = rx_context->rx_data_end_pos_;
            uint16_t size = rx_context->rx_buffer_size_;

            if (instance->backend_handle_.stream_buffer_handle_ == NULL) {
                return;
            }

            if (start < end) {
                sendSegmentToBufferFromISR(instance,
                                           rx_context->rx_buffer_ptr_,
                                           size,
                                           start,
                                           end,
                                           &xHigherPriorityTaskWoken);
            } else if (start > end) {
                sendSegmentToBufferFromISR(instance,
                                           rx_context->rx_buffer_ptr_,
                                           size,
                                           start,
                                           size,
                                           &xHigherPriorityTaskWoken);
                sendSegmentToBufferFromISR(instance,
                                           rx_context->rx_buffer_ptr_,
                                           size,
                                           0U,
                                           end,
                                           &xHigherPriorityTaskWoken);
            } else {
                return;
            }
            break;
        }

        case MODULE_SERIAL_STREAM_BACKEND_DMA_RING_BUFFER:
            if (isDMARingBufferNotifyHandleValid(instance) == false) {
                return;
            }

            instance->rx_dma_buffer_ptr_ = rx_context->rx_buffer_ptr_;
            instance->rx_dma_buffer_size_ = rx_context->rx_buffer_size_;

            instance->rx_write_bytes_ += rx_context->rx_data_len_;
            instance->rx_total_bytes_ += rx_context->rx_data_len_;

            notifyDMARingBufferReceiverFromISR(instance, &xHigherPriorityTaskWoken);
            break;
    }

    // 如果唤醒了更高优先级任务，触发调度
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

bool moduleSerialStreamInit(moduleSerialStream_t *instance, const moduleSerialStreamConfig_t *config)
{
    if (instance == NULL || config == NULL) {
        return false;
    }

    if (config->uart_instance_ == NULL) {
        return false;
    }

    if (isBackendValid(config->backend_) == false ||
        isNotifyBackendValid(config->notify_backend_) == false) {
        return false;
    }

    switch (config->backend_) {
        case MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER:
            if (config->backend_handle_.stream_buffer_handle_ == NULL) {
                return false;
            }

            // StreamBuffer 模式不需要额外通知后端，避免误配 task/semaphore 语义不清
            if (config->notify_backend_ != MODULE_SERIAL_STREAM_NOTIFY_NONE) {
                return false;
            }
            break;

        case MODULE_SERIAL_STREAM_BACKEND_DMA_RING_BUFFER:
            // DMA ring buffer 模式必须显式指定通知方式，并且对应 handle 必须有效
            if (isDMARingBufferNotifyConfigValid(config) == false) {
                return false;
            }
            break;

        default:
            return false;
    }

    memset(instance, 0, sizeof(moduleSerialStream_t));
    instance->uart_instance_ = config->uart_instance_;
    instance->backend_ = config->backend_;
    instance->backend_handle_ = config->backend_handle_;
    instance->notify_backend_ = config->notify_backend_;

    bspUARTRxEventCallbackRegister(instance->uart_instance_, instance, uartRxCallbackFromISR);

    return true;
}

uint32_t moduleSerialStreamRead(moduleSerialStream_t *instance,
                                uint8_t *data_out,
                                uint32_t max_length,
                                TickType_t timeout_tick)
{
    if (instance == NULL || data_out == NULL || max_length == 0U) {
        return 0U;
    }

    switch (instance->backend_) {
        case MODULE_SERIAL_STREAM_BACKEND_STREAM_BUFFER:
            uint32_t read_bytes;

            if (instance->backend_handle_.stream_buffer_handle_ == NULL) {
                return 0U;
            }

            read_bytes = (uint32_t)xStreamBufferReceive(instance->backend_handle_.stream_buffer_handle_,
                                                        data_out,
                                                        max_length,
                                                        timeout_tick);
            instance->rx_accept_bytes_ += read_bytes;
            return read_bytes;

        case MODULE_SERIAL_STREAM_BACKEND_DMA_RING_BUFFER:
            // DMA ring buffer 模式下，这里不阻塞等待；task notify / semaphore take 交给上层任务处理
            (void)timeout_tick;
            return readDataDMARingBuffer(instance, data_out, max_length);

        default:
            break;
    }

    return 0U;
}
