#include "can.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_def.h"
#include "bsp_can.h"
#include "bsp_can_private.h"
#include "stm32f4xx_hal_can.h"

typedef struct can_instance
{
    CAN_HandleTypeDef *can_handle_;

    const char *name_;

    bspCANRxRoute_t rx_route_[BSP_CAN_SINGLE_MAX_DEVICE_NUM];
    uint8_t rx_route_count_;
    uint8_t filter_configured_route_count_;
    bool is_started_;

    bspCANErrorCallback_f error_callback_;
} bspCANInstance_t;

static uint8_t can_memory_index_ = 0;
static bspCANInstance_t can_instance_memory_[BSP_CAN_MAX_INSTANCE_NUM] = {NULL};

static bspCANStatus_e bspCANGetStatusFromHAL(HAL_StatusTypeDef status_hal)
{
    switch (status_hal) {
        case HAL_OK:
            return BSP_CAN_OK;
        case HAL_ERROR:
            return BSP_CAN_ERROR;
        case HAL_BUSY:
            return BSP_CAN_BUSY;
        case HAL_TIMEOUT:
            return BSP_CAN_TIMEOUT;
        default:
            break;
    }
    return BSP_CAN_ERROR;
}

static bspCANInstance_t *bspCANFindInstanceByHandle(CAN_HandleTypeDef *can_handle)
{
    for (size_t i = 0; i < can_memory_index_; i ++) {
        if (can_instance_memory_[i].can_handle_ == can_handle) {
            return &can_instance_memory_[i];
        }
    }
    return NULL;
}

static bspCANRxRoute_t *bspCANFindRxRouteByHeader(bspCANInstance_t *instance,
                                          const CAN_RxHeaderTypeDef *rx_message_header_hal)
{
    uint32_t rx_message_id = 0;

    if (instance == NULL || rx_message_header_hal == NULL) {
        return NULL;
    }

    if (rx_message_header_hal->IDE == CAN_ID_STD) {
        rx_message_id = rx_message_header_hal->StdId;
    } else if (rx_message_header_hal->IDE == CAN_ID_EXT) {
        rx_message_id = rx_message_header_hal->ExtId;
    } else {
        return NULL;
    }

    for (size_t i = 0; i < instance->rx_route_count_; i ++) {
        bspCANRxRoute_t *rx_route = &instance->rx_route_[i];
        if (rx_route->route_ide_ != rx_message_header_hal->IDE) {
            continue;
        }
        if (rx_route->route_id_ != rx_message_id) {
            continue;
        }
        return rx_route;
    }

    return NULL;
}

static void bspCANPackRxMessage(const CAN_RxHeaderTypeDef *rx_message_header_hal,
                                const uint8_t *rx_message_data,
                                bspCANMessage_t *rx_message)
{
    uint32_t copy_len = 0;

    if (rx_message_header_hal == NULL || rx_message_data == NULL || rx_message == NULL) {
        return;
    }

    memset(rx_message, 0, sizeof(bspCANMessage_t));

    if (rx_message_header_hal->IDE == CAN_ID_STD) {
        rx_message->message_header_.message_id_ = rx_message_header_hal->StdId;
    } else {
        rx_message->message_header_.message_id_ = rx_message_header_hal->ExtId;
    }
    rx_message->message_header_.message_ide_ = rx_message_header_hal->IDE;
    rx_message->message_header_.message_rtr_ = rx_message_header_hal->RTR;
    rx_message->message_header_.message_dlc_ = rx_message_header_hal->DLC;

    copy_len = rx_message_header_hal->DLC;
    if (copy_len > 8U) {
        copy_len = 8U;
    }
    memcpy(rx_message->message_data_, rx_message_data, copy_len);
}

bspCANInstance_t *bspCANInit(const bspCANConfig_t *config)
{
    if (config == NULL) {
        return  NULL;
    }

    if (can_memory_index_ >= BSP_CAN_MAX_INSTANCE_NUM) {
        return NULL;
    }

    bspCANInstance_t *instance = &can_instance_memory_[can_memory_index_];
    memset(instance, 0, sizeof(bspCANInstance_t));
    instance->can_handle_ = config->can_handle_;
    instance->name_ = config->name_;
    instance->rx_route_count_ = 0;
    instance->filter_configured_route_count_ = 0;
    instance->is_started_ = false;

    can_memory_index_ ++;

    return instance;
}

// 注意，owner也许并不知道会触发RX0 还是RX1回调？
void bspCANRxCallbackRegister(bspCANInstance_t *instance, bspCANRxRoute_t rx_route)
{
    if (instance == NULL || 
        rx_route.route_owner_ == NULL || 
        rx_route.route_rx_callback_ == NULL ||
        instance->rx_route_count_ >= BSP_CAN_SINGLE_MAX_DEVICE_NUM) {
        return;
    }

    instance->rx_route_[instance->rx_route_count_] = rx_route;
    instance->rx_route_count_ ++;
}

void bspCANErrorCallbackRegister(bspCANInstance_t *instance, bspCANErrorCallback_f callback)
{
    instance->error_callback_ = callback;
}

// 暂时把过滤器配置写死
// 列表模式，只考虑标准帧
bspCANStatus_e bspCANSetFilter(bspCANInstance_t *instance)
{   
    if (instance == NULL || instance->can_handle_ == NULL || instance->rx_route_count_ == 0) {
        return BSP_CAN_ERROR;
    }

    if (instance->filter_configured_route_count_ == instance->rx_route_count_) {
        return BSP_CAN_OK;
    }

    if (instance->is_started_ == true) {
        return BSP_CAN_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    CAN_FilterTypeDef filter_config;
    for (size_t i = 0; i < instance->rx_route_count_; i ++) {
        memset(&filter_config, 0, sizeof(CAN_FilterTypeDef)); // 清零防止意外数据存在
        filter_config.FilterIdLow = (instance->rx_route_[i].route_id_) << 5; // 注意，是低5位补0
        filter_config.FilterIdHigh = (instance->rx_route_[i].route_id_) << 5;
        filter_config.FilterMaskIdLow = (instance->rx_route_[i].route_id_) << 5;
        filter_config.FilterMaskIdHigh = (instance->rx_route_[i].route_id_) << 5;
        filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filter_config.FilterBank = i; // 这是很浪费的写法，但暂时这样
        filter_config.FilterMode = CAN_FILTERMODE_IDLIST;
        filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
        filter_config.FilterActivation = CAN_FILTER_ENABLE;
        // bxCAN由主CAN1和从CAN2，共用0-27总共28个filterbank，这个参数是用来划定从[SlaveStartFilterBank, 27]之间的filterbank给CAN2
        filter_config.SlaveStartFilterBank = 14; 
        
        status_hal = HAL_CAN_ConfigFilter(instance->can_handle_, &filter_config);
        if (status_hal != HAL_OK) {
            return bspCANGetStatusFromHAL(status_hal);
        }
    }

    instance->filter_configured_route_count_ = instance->rx_route_count_;

    return BSP_CAN_OK;
}

bspCANStatus_e bspCANStart(bspCANInstance_t *instance)
{
    if (instance == NULL || instance->can_handle_ == NULL) {
        return BSP_CAN_ERROR;
    }

    if (instance->is_started_ == true) {
        return BSP_CAN_OK;
    }

    HAL_StatusTypeDef status_hal;

    // 启动CAN外设
    status_hal = HAL_CAN_Start(instance->can_handle_);
    if (status_hal != HAL_OK) {
        return bspCANGetStatusFromHAL(status_hal);
    }
    // 开启FIFO0消息挂起中断，只要FIFO0有至少一个报文，就触发中断
    status_hal = HAL_CAN_ActivateNotification(instance->can_handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status_hal != HAL_OK) {
        return bspCANGetStatusFromHAL(status_hal);
    }
    // 开启FIFO1消息挂起中断，只要FIFO1有至少一个报文，就触发中断
    status_hal = HAL_CAN_ActivateNotification(instance->can_handle_, CAN_IT_RX_FIFO1_MSG_PENDING);
    if (status_hal != HAL_OK) {
        return bspCANGetStatusFromHAL(status_hal);
    }

    instance->is_started_ = true;

    return BSP_CAN_OK;
}

// 无阻塞，之后可能可以加一个阻塞等待，但是这个等待给上层处理或许更好？
bspCANStatus_e bspCANTransmit(bspCANInstance_t *instance, const bspCANMessage_t *tx_message)
{
    if (instance == NULL || 
        instance->can_handle_ == NULL || 
        tx_message == NULL || 
        tx_message->message_header_.message_dlc_ > 8) { // 注意载荷为0没关系
        return BSP_CAN_ERROR;
    }

    HAL_StatusTypeDef status_hal;
    // 将消息添加到邮箱前，必须先检查3个邮箱是否有空位
    if (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle_) == 0) {
        // 没有空位
        return BSP_CAN_BUSY;
    }
    
    CAN_TxHeaderTypeDef tx_message_header_hal;
    if (tx_message->message_header_.message_ide_ == CAN_ID_STD) {
        tx_message_header_hal.StdId = tx_message->message_header_.message_id_;
    } else {
        tx_message_header_hal.ExtId = tx_message->message_header_.message_id_;
    }
    tx_message_header_hal.IDE = tx_message->message_header_.message_ide_;
    tx_message_header_hal.RTR = tx_message->message_header_.message_rtr_;
    tx_message_header_hal.DLC = tx_message->message_header_.message_dlc_;
    tx_message_header_hal.TransmitGlobalTime = DISABLE;
    uint32_t tx_mailbox = 0;

    status_hal = HAL_CAN_AddTxMessage(instance->can_handle_, 
                                    &tx_message_header_hal, 
                                    tx_message->message_data_, 
                                    &tx_mailbox);
    
    return bspCANGetStatusFromHAL(status_hal);
}

static void bspCANFifoxMsgPendingCallback(CAN_HandleTypeDef *hcan, uint32_t RX_FIFOx)
{
    bspCANInstance_t *instance = bspCANFindInstanceByHandle(hcan);
    if (instance == NULL) {
        return;
    }

    CAN_RxHeaderTypeDef rx_message_header_hal = {0};
    uint8_t rx_message_data[8] = {0};
    bspCANMessage_t rx_message = {0};
    bspCANRxRoute_t *rx_route = NULL;
    
    // 循环，把fifox中的报文全部处理完
    while (HAL_CAN_GetRxFifoFillLevel(hcan, RX_FIFOx)) {
        // 必须从FIFO中取走消息
        if (HAL_CAN_GetRxMessage(hcan, RX_FIFOx, &rx_message_header_hal, rx_message_data) != HAL_OK) {
            return;
        }
        bspCANPackRxMessage(&rx_message_header_hal, rx_message_data, &rx_message);

        // 按照报文ID调用对应的回调函数
        rx_route = bspCANFindRxRouteByHeader(instance, &rx_message_header_hal);
        if (rx_route != NULL && rx_route->route_rx_callback_ != NULL) {
            rx_route->route_rx_callback_(rx_route->route_owner_, &rx_message);
        }
    } 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    bspCANFifoxMsgPendingCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    bspCANFifoxMsgPendingCallback(hcan, CAN_RX_FIFO1);
}

