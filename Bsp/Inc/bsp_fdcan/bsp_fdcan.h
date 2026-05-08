#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_def.h"

typedef enum
{
    BSP_FDCAN_OK = 0,
    BSP_FDCAN_ERROR,
    BSP_FDCAN_BUSY,
    BSP_FDCAN_TIMEOUT
} bspFDCANStatus_e;

typedef enum
{
    BSP_FDCAN_ID_STD = 0,
    BSP_FDCAN_ID_EXT,
} bspFDCANIdType_e; // 报文ID类型，标准/扩展

typedef enum
{
    BSP_FDCAN_FRAME_CLASSIC = 0, // 经典帧格式
    BSP_FDCAN_FRAME_FD_NO_BRS, // FD帧，无数据段速率切换
    BSP_FDCAN_FRAME_FD_BRS, // FD帧，带数据段速率切换
} bspFDCANFrameFormat_e; // 报文格式

typedef enum
{
    BSP_FDCAN_DATA_FRAME = 0, // 数据帧
    BSP_FDCAN_REMOTE_FRAME, // 遥控帧，请注意CAN FD 帧格式不支持 remote frame,但 FDCAN 外设在 Classical CAN 格式下可以发 remote frame。
} bspFDCANFrameType_e; // 报文类型

typedef struct fdcan_message_header
{
    uint32_t message_id_; // 报文ID，11位或25位
    bspFDCANIdType_e message_ide_; // 0为标准帧
    bspFDCANFrameType_e message_rtr_; // 0为数据帧,请注意FDCAN不支持遥控帧，这里是为了兼容classical CAN
    bspFDCANFrameFormat_e message_format_; // classical、FD 、FD + BRS
    uint8_t message_length_; // 0-64字节，不可以随意配置,请使用 获取支持的长度
    
    uint8_t message_brs_; // BRS位，原则上，不能在这里配置，只能表示
    uint8_t message_esi_; // ESIw位，发送节点错误指示
} bspFDCANMessageHeader_t;

typedef struct fdcan_message
{
    bspFDCANMessageHeader_t message_header_;

    uint8_t message_data_[BSP_FDCAN_MAX_DATA_LENGTH]; // FDCAN，载荷最大为64字节
} bspFDCANMessage_t;

typedef struct fdcan_instance bspFDCANInstance_t;

// FDCAN 是报文，不是字节流,后面无论接收还是发送，中心都应该是 bspFDCANMessage_t，不是裸 buffer。
typedef void (*bspFDCANRxCallback_f)(void *owner, const bspFDCANMessage_t *rx_message);
typedef void (*bspFDCANErrorCallback_f)(void *owner);

// 一个FDCAN可以挂载多个设备，每个设备都需要注册自己的回调函数
typedef struct fdcan_rx_route
{
    uint32_t route_id_; // 根据报文id，找到对应的设备owner
    bspFDCANIdType_e route_ide_;
    uint32_t route_id_mask_; // 掩码，用于匹配一个范围内的id，为0时表示精确匹配

    bspFDCANRxCallback_f route_rx_callback_;
    void *route_owner_;
} bspFDCANRxRoute_t;

// 请注意，这个函数暂时没有返回值，调用者不知到到底有没有注册成功，需要修改！！！
void bspFDCANRxCallbackRegister(bspFDCANInstance_t *instance, bspFDCANRxRoute_t rx_route);
void bspFDCANErrorCallbackRegister(bspFDCANInstance_t *instance, bspFDCANErrorCallback_f callback);

bspFDCANStatus_e bspFDCANSetFilter(bspFDCANInstance_t *instance);
bspFDCANStatus_e bspFDCANStart(bspFDCANInstance_t *instance);
bspFDCANStatus_e bspFDCANTransmit(bspFDCANInstance_t *instance, const bspFDCANMessage_t *tx_message);

bool bspFDCANDataLengthIsValid(uint8_t data_length);
// 向上取整获得最近合法payload长度
uint8_t bspFDCANGetValidDataLengthCeil(uint8_t data_length);