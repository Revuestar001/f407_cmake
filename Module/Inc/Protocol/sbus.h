#pragma once
#include <stdint.h>
#include <stdbool.h>

#define PROTOCOL_SBUS_CHANNEL_NUM 16
#define PROTOCOL_SBUS_FRAME_SIZE 25
#define PROTOCOL_SBUS_FRAME_START_BYTE 0x0FU
#define PROTOCOL_SBUS_FRAME_END_BYTE 0x00U

#define PROTOCOL_SBUS_CHANNEL_RAW_MAX_VALUE  1811
#define PROTOCOL_SBUS_CHANNEL_RAW_MIN_VALUE  172

typedef enum
{
    PROTOCOL_SBUS_FEED_NO_FRAME = 0,
    PROTOCOL_SBUS_FEED_FRAME_OK,
    PROTOCOL_SBUS_FEED_FRAME_ERROR
} protocolSBUSFeedResult_e;

// 一个完整的sbus消息包含22个字节的有效数据
// 数据低位在前，11位为1个完整数据
typedef struct sbus_data_frame
{
    uint16_t channel_data_raw_[PROTOCOL_SBUS_CHANNEL_NUM];
    uint16_t channel_data_us_[PROTOCOL_SBUS_CHANNEL_NUM]; // 正则到[1000, 2000]us
    // 两个数字通道位
    bool channel_17_;
    bool channel_18_;
    // 两个状态位
    bool frame_lost_flag_;
    bool failsafe_activate_flag_;
} protocolSBUSDataFrame_t;

// 没有加入更强的重同步，比如 IDLE/gap、尾字节多种合法值、错误后回退重找头，这里只使用synced_作为最简单的状态机
typedef struct sbus_data_praser
{
    uint8_t frame_buffer_[PROTOCOL_SBUS_FRAME_SIZE]; // sbus需要25字节凑够一帧，这里和DMA缓冲以及软件ringbuffer没关系
    uint8_t frame_pos_; // 记录当前解析到一帧的哪个位置
    bool synced_; // 记录是否找到帧头，因为传输时是字节流不是完整帧，一旦找到帧头，说明已同步
} protocolSBUSDataPraser_t;

// sbus_praser只提供协议解析，本身不提供内存，也不会自己创建实例，不会注册回调函数
// 因为sbus_praser不是板级资源，上层需要使用sbus解析时才应该创建
void protocolSBUSPraserInit(protocolSBUSDataPraser_t *instance);
// 处理一个字节
protocolSBUSFeedResult_e protocolSBUSPraserFeedByte(protocolSBUSDataPraser_t *instance, uint8_t byte, protocolSBUSDataFrame_t *sbus_frame_out);
// 输入多字节，寻找多个帧，返回找到的帧个数
uint16_t protocolSBUSPraserFeedBuffer(protocolSBUSDataPraser_t *instance,
                                      const uint8_t *buffer_ptr,
                                      uint16_t buffer_size,
                                      protocolSBUSDataFrame_t *sbus_frame_out_array,
                                      uint16_t sbus_frame_out_array_size);
// 输入多字节，寻找多个帧，但只给出最后一帧有效帧
protocolSBUSFeedResult_e protocolSBUSPraserFeedBufferLastFrame(protocolSBUSDataPraser_t *instance,
                                                               const uint8_t *buffer_ptr,
                                                               uint16_t buffer_size,
                                                               protocolSBUSDataFrame_t *sbus_last_frame_out);
