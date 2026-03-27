#pragma once
#include <stdbool.h>
#include <stdint.h>

// 仅适用于单生产者-单消费者
typedef struct ring_buffer
{
    uint8_t *ring_buffer_ptr_;
    uint16_t ring_buffer_size_;

    // 因为会在中断中修改，必须加volatile
    volatile uint16_t head_index_; // 指向ringbuffer中的写索引，即下一个需要写位置
    volatile uint16_t tail_index_; // 指向ringbuffer中的读索引，即下一个需要读位置
} utilRingBuffer_t;

// ring buffer只负责绑定缓冲数组，本身不提供内存，也不会自己创建ring buffer对象
void utilRingBufferInit(utilRingBuffer_t *instance, uint8_t *buffer_ptr, uint16_t buffer_size);

bool utilRingBufferIsEmpty(const utilRingBuffer_t *instance);
bool utilRingBufferIsFull(const utilRingBuffer_t *instance);
uint16_t utilRingBufferGetUsedSize(const utilRingBuffer_t *instance);
// 实际上，全满时也有一个索引处可写入数据，但并不能使用，否则head == tail会判定缓冲区空
uint16_t utilRingBufferGetFreeSize(const utilRingBuffer_t *instance);
bool utilRingBufferPushByte(utilRingBuffer_t *instance, uint8_t data);
bool utilRingBufferPopByte(utilRingBuffer_t *instance, uint8_t *data_popped_out);
bool utilRingBufferPushBytesFromBuffer(utilRingBuffer_t *instance, const uint8_t *buffer_ptr, uint16_t data_length);