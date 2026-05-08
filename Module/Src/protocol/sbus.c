#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "sbus.h"

static void protocolSBUSPraserReset(protocolSBUSDataPraser_t *instance)
{
    memset(instance, 0, sizeof(protocolSBUSDataPraser_t));
}

static uint16_t protocolSBUSChannelNormalize(uint16_t raw)
{
    if (raw <= PROTOCOL_SBUS_CHANNEL_RAW_MIN_VALUE) {
        return 1000U;
    }

    if (raw >= PROTOCOL_SBUS_CHANNEL_RAW_MAX_VALUE) {
        return 2000U;
    }

    return (uint16_t)((((uint32_t)(raw - PROTOCOL_SBUS_CHANNEL_RAW_MIN_VALUE)) * 1000U) / (PROTOCOL_SBUS_CHANNEL_RAW_MAX_VALUE - PROTOCOL_SBUS_CHANNEL_RAW_MIN_VALUE) + 1000U);
}

static bool protocolSBUSPraserDecodeFrame(protocolSBUSDataPraser_t *instance, protocolSBUSDataFrame_t *sbus_data_decoded)
{
    if (instance == NULL || sbus_data_decoded == NULL || instance->frame_pos_ != PROTOCOL_SBUS_FRAME_SIZE) {
        return false;
    }

    if (instance->frame_buffer_[0] != PROTOCOL_SBUS_FRAME_START_BYTE ||
        instance->frame_buffer_[PROTOCOL_SBUS_FRAME_SIZE - 1] != PROTOCOL_SBUS_FRAME_END_BYTE) {
        return false;
    }

    // 一个通道11bits，从第二个字节开始
    sbus_data_decoded->channel_data_raw_[0]  = ((uint16_t)instance->frame_buffer_[1]       | ((uint16_t)instance->frame_buffer_[2]  << 8)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[1]  = (((uint16_t)instance->frame_buffer_[2] >> 3) | ((uint16_t)instance->frame_buffer_[3]  << 5)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[2]  = (((uint16_t)instance->frame_buffer_[3] >> 6) | ((uint16_t)instance->frame_buffer_[4]  << 2) | ((uint16_t)instance->frame_buffer_[5]  << 10)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[3]  = (((uint16_t)instance->frame_buffer_[5] >> 1) | ((uint16_t)instance->frame_buffer_[6]  << 7)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[4]  = (((uint16_t)instance->frame_buffer_[6] >> 4) | ((uint16_t)instance->frame_buffer_[7]  << 4)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[5]  = (((uint16_t)instance->frame_buffer_[7] >> 7) | ((uint16_t)instance->frame_buffer_[8]  << 1) | ((uint16_t)instance->frame_buffer_[9]  << 9)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[6]  = (((uint16_t)instance->frame_buffer_[9] >> 2) | ((uint16_t)instance->frame_buffer_[10] << 6)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[7]  = (((uint16_t)instance->frame_buffer_[10] >> 5) | ((uint16_t)instance->frame_buffer_[11] << 3)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[8]  = ((uint16_t)instance->frame_buffer_[12]      | ((uint16_t)instance->frame_buffer_[13] << 8)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[9]  = (((uint16_t)instance->frame_buffer_[13] >> 3) | ((uint16_t)instance->frame_buffer_[14] << 5)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[10] = (((uint16_t)instance->frame_buffer_[14] >> 6) | ((uint16_t)instance->frame_buffer_[15] << 2) | ((uint16_t)instance->frame_buffer_[16] << 10)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[11] = (((uint16_t)instance->frame_buffer_[16] >> 1) | ((uint16_t)instance->frame_buffer_[17] << 7)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[12] = (((uint16_t)instance->frame_buffer_[17] >> 4) | ((uint16_t)instance->frame_buffer_[18] << 4)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[13] = (((uint16_t)instance->frame_buffer_[18] >> 7) | ((uint16_t)instance->frame_buffer_[19] << 1) | ((uint16_t)instance->frame_buffer_[20] << 9)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[14] = (((uint16_t)instance->frame_buffer_[20] >> 2) | ((uint16_t)instance->frame_buffer_[21] << 6)) & 0x07FFU;
    sbus_data_decoded->channel_data_raw_[15] = (((uint16_t)instance->frame_buffer_[21] >> 5) | ((uint16_t)instance->frame_buffer_[22] << 3)) & 0x07FFU;
    // 正则到[1000, 2000]us
    for (size_t i = 0; i < PROTOCOL_SBUS_CHANNEL_NUM; i ++) {
        sbus_data_decoded->channel_data_us_[i] = protocolSBUSChannelNormalize(sbus_data_decoded->channel_data_raw_[i]);
    }
    // 注意，这里实现和CSDN不同
    // 两个数字通道
    sbus_data_decoded->channel_17_ = (instance->frame_buffer_[23] & (0x01U << 0)) != 0U;
    sbus_data_decoded->channel_18_ = (instance->frame_buffer_[23] & (0x01U << 1)) != 0U;
    // 两个标志位
    sbus_data_decoded->frame_lost_flag_ = (instance->frame_buffer_[23] & (0x01U << 2)) != 0U;
    sbus_data_decoded->failsafe_activate_flag_ = (instance->frame_buffer_[23] & (0x01U << 3)) != 0U;

    return true;
}

// sbus_praser只提供协议解析，本身不提供内存，也不会自己创建实例，不会注册回调函数
// 因为sbus_praser不是板级资源，上层需要使用sbus解析时才应该创建
void protocolSBUSPraserInit(protocolSBUSDataPraser_t *instance)
{
    if (instance == NULL) {
        return;
    }

    protocolSBUSPraserReset(instance);
}

// 处理一个字节
protocolSBUSFeedResult_e protocolSBUSPraserFeedByte(protocolSBUSDataPraser_t *instance, uint8_t byte, protocolSBUSDataFrame_t *sbus_frame_out)
{
    if (instance == NULL || sbus_frame_out == NULL) {
        return PROTOCOL_SBUS_FEED_FRAME_ERROR;
    }

    if (instance->synced_ == false) {
        // 还没同步
        if (byte != PROTOCOL_SBUS_FRAME_START_BYTE) {
            return PROTOCOL_SBUS_FEED_NO_FRAME;
        } else {
            // 找到帧头
            instance->frame_pos_ = 0;
            instance->frame_buffer_[instance->frame_pos_ ++] = byte;
            instance->synced_ = true;

            return PROTOCOL_SBUS_FEED_NO_FRAME;
        }
    }

    if (instance->frame_pos_ >= PROTOCOL_SBUS_FRAME_SIZE) {
        protocolSBUSPraserReset(instance);
        return PROTOCOL_SBUS_FEED_FRAME_ERROR;
    }

    instance->frame_buffer_[instance->frame_pos_ ++] = byte;

    if (instance->frame_pos_ == PROTOCOL_SBUS_FRAME_SIZE) {
        if (protocolSBUSPraserDecodeFrame(instance, sbus_frame_out) == true) {
            // 找到完整的一帧，也清空praser的状态
            protocolSBUSPraserReset(instance);
            return PROTOCOL_SBUS_FEED_FRAME_OK;
        } else {
            protocolSBUSPraserReset(instance);
            return PROTOCOL_SBUS_FEED_FRAME_ERROR;
        }
    }

    return PROTOCOL_SBUS_FEED_NO_FRAME;
}

// 输入多字节，寻找多个帧，返回找到的帧个数
uint16_t protocolSBUSPraserFeedBuffer(protocolSBUSDataPraser_t *instance,
                                      const uint8_t *buffer_ptr,
                                      uint16_t buffer_size,
                                      protocolSBUSDataFrame_t *sbus_frame_out_array,
                                      uint16_t sbus_frame_out_array_size)
{
    uint16_t frame_out_count = 0;

    if (instance == NULL || buffer_ptr == NULL || sbus_frame_out_array == NULL) {
        return 0;
    }

    for (uint16_t i = 0; i < buffer_size; i ++) {
        protocolSBUSDataFrame_t temp_frame;
        protocolSBUSFeedResult_e result;

        if (frame_out_count < sbus_frame_out_array_size) {
            result = protocolSBUSPraserFeedByte(instance, buffer_ptr[i], &sbus_frame_out_array[frame_out_count]);
            if (result == PROTOCOL_SBUS_FEED_FRAME_OK) {
                frame_out_count ++;
            }
        } else {
            // 即使传入的保存sbus帧的数组满了，也必须继续进行解析，因为是流式传输，中途解析中断会造成sbus_praser内部状态错位
            result = protocolSBUSPraserFeedByte(instance, buffer_ptr[i], &temp_frame);
        }
    }

    return frame_out_count;
}

// 输入多字节，寻找多个帧，但只返回最后一帧有效帧
protocolSBUSFeedResult_e protocolSBUSPraserFeedBufferLastFrame(protocolSBUSDataPraser_t *instance,
                                                               const uint8_t *buffer_ptr,
                                                               uint16_t buffer_size,
                                                               protocolSBUSDataFrame_t *sbus_last_frame_out)
{
    if (instance == NULL || buffer_ptr == NULL || sbus_last_frame_out == NULL) {
        return PROTOCOL_SBUS_FEED_FRAME_ERROR;
    }
    
    protocolSBUSFeedResult_e feed_result = PROTOCOL_SBUS_FEED_NO_FRAME;
    protocolSBUSDataFrame_t last_frame_out = {0};

    for (uint16_t i = 0; i < buffer_size; i ++) {
        protocolSBUSFeedResult_e result = protocolSBUSPraserFeedByte(instance, buffer_ptr[i], &last_frame_out);

        if (result == PROTOCOL_SBUS_FEED_FRAME_OK) {
            *sbus_last_frame_out = last_frame_out;
            feed_result = PROTOCOL_SBUS_FEED_FRAME_OK;
        } else if (result == PROTOCOL_SBUS_FEED_FRAME_ERROR) {
            if (feed_result != PROTOCOL_SBUS_FEED_FRAME_OK) {
                feed_result = PROTOCOL_SBUS_FEED_FRAME_ERROR;
            }
        }
    }

    return feed_result;
}

