#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "bsp_def.h"

typedef enum
{
    BSP_CAN_OK = 0,
    BSP_CAN_ERROR,
    BSP_CAN_BUSY,
    BSP_CAN_TIMEOUT
} bspCANStatus_e;

typedef struct can_message_header
{
    uint32_t message_id_; // 11位或25位
    uint32_t message_ide_; // 0为标准帧
    uint32_t message_rtr_; // 0为数据帧
    uint32_t message_dlc_; // 0-8字节
} bspCANMessageHeader_t;

typedef struct can_message
{
    bspCANMessageHeader_t message_header_;

    uint8_t message_data_[8]; // 标准CAN，载荷最大为8字节
} bspCANMessage_t;

typedef struct can_instance bspCANInstance_t;

// CAN 是报文，不是字节流,后面无论接收还是发送，中心都应该是 bspCANMessage_t，不是裸 buffer。
typedef void (*bspCANRxCallback_f)(void *owner, const bspCANMessage_t *rx_message);
typedef void (*bspCANErrorCallback_f)(void *owner);

// 一个CAN可以挂载多个设备，每个设备都需要注册自己的回调函数
typedef struct can_rx_route
{
    uint32_t route_id_; // 根据报文id，找到对应的设备owner
    uint32_t route_ide_;
    uint32_t route_id_mask_; // 掩码，用于匹配一个范围内的id，暂时用不上

    bspCANRxCallback_f route_rx_callback_;
    void *route_owner_;
} bspCANRxRoute_t;

void bspCANRxCallbackRegister(bspCANInstance_t *instance, bspCANRxRoute_t rx_route);
void bspCANErrorCallbackRegister(bspCANInstance_t *instance, bspCANErrorCallback_f callback);

bspCANStatus_e bspCANSetFilter(bspCANInstance_t *instance);
bspCANStatus_e bspCANStart(bspCANInstance_t *instance);
bspCANStatus_e bspCANTransmit(bspCANInstance_t *instance, const bspCANMessage_t *tx_message);
