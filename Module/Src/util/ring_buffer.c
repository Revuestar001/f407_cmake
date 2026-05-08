#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ring_buffer.h"

static uint16_t utilRingBufferGetNextIndex(const utilRingBuffer_t *instance, uint16_t index)
{
    index ++;
    if (index >= instance->ring_buffer_size_) {
        index = 0;
    }
    return index;
}

static void utilRingBufferPushByteNoCheck(utilRingBuffer_t *instance, uint8_t data)
{
    uint16_t head_index = instance->head_index_;
    uint16_t next_head_index = utilRingBufferGetNextIndex(instance, head_index);
    instance->ring_buffer_ptr_[head_index] = data;
    instance->head_index_ = next_head_index;
}

// ring buffer只负责绑定缓冲数组，本身不提供内存，也不会自己创建ring buffer对象
void utilRingBufferInit(utilRingBuffer_t *instance, uint8_t *buffer_ptr, uint16_t buffer_size)
{
    if (instance == NULL || buffer_ptr == NULL || buffer_size == 0) {
        return ;
    }

    instance->ring_buffer_ptr_ = buffer_ptr;
    instance->ring_buffer_size_ = buffer_size;
    instance->head_index_ = 0;
    instance->tail_index_ = 0;
}

bool utilRingBufferIsEmpty(const utilRingBuffer_t *instance)
{
    if (instance == NULL) {
        return false;
    }

    return instance->head_index_ == instance->tail_index_;
}

bool utilRingBufferIsFull(const utilRingBuffer_t *instance)
{
    if (instance == NULL || instance->ring_buffer_size_ == 0) {
        return true;
    }

    uint16_t head_index = instance->head_index_;
    uint16_t next_head_index = utilRingBufferGetNextIndex(instance, head_index);

    return next_head_index == instance->tail_index_;
}

uint16_t utilRingBufferGetUsedSize(const utilRingBuffer_t *instance)
{
    if (instance == NULL) {
        return UINT16_MAX;
    }

    if (instance->head_index_ >= instance->tail_index_) {
        return instance->head_index_ - instance->tail_index_;
    } else {
        return (instance->ring_buffer_size_ - instance->tail_index_) + instance->head_index_;
    }
}

// 实际上，全满时也有一个索引处可写入数据，但并不能使用，否则head == tail会判定缓冲区空
uint16_t utilRingBufferGetFreeSize(const utilRingBuffer_t *instance)
{
    if (instance == NULL) {
        return 0;
    }

    return instance->ring_buffer_size_ - utilRingBufferGetUsedSize(instance) - 1;
}

bool utilRingBufferPushByte(utilRingBuffer_t *instance, uint8_t data)
{
    if (instance == NULL) {
        return false;
    }

    if (utilRingBufferIsFull(instance)) {
        return false;
    }

    uint16_t head_index = instance->head_index_;
    uint16_t next_head_index = utilRingBufferGetNextIndex(instance, head_index);
    instance->ring_buffer_ptr_[head_index] = data;
    instance->head_index_ = next_head_index;

    return true;
}

bool utilRingBufferPopByte(utilRingBuffer_t *instance, uint8_t *data_popped_out)
{
    if (instance == NULL || data_popped_out == NULL) {
        return false;
    }

    if (utilRingBufferIsEmpty(instance)) {
        return false;
    }

    uint16_t tail_index = instance->tail_index_;
    uint16_t next_tail_index = utilRingBufferGetNextIndex(instance, tail_index);
    *data_popped_out = instance->ring_buffer_ptr_[tail_index];
    instance->tail_index_ = next_tail_index;

    return true;
}

bool utilRingBufferPushBytesFromBuffer(utilRingBuffer_t *instance, const uint8_t *buffer_ptr, uint16_t data_length)
{
    if (instance == NULL || buffer_ptr == NULL) {
        return false;
    }

    if (utilRingBufferGetFreeSize(instance) < data_length) {
        return false;
    }

    for (size_t i = 0; i < data_length; i ++) {
        utilRingBufferPushByteNoCheck(instance, buffer_ptr[i]);
    }

    return true;
}
