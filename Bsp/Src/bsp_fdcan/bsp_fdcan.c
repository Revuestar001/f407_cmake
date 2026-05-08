#include "fdcan.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bsp_def.h"
#include "bsp_fdcan.h"
#include "bsp_fdcan_private.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_fdcan.h"

typedef struct fdcan_instance
{
    FDCAN_HandleTypeDef *fdcan_handle_;

    const char *name_;

    bspFDCANRxRoute_t rx_route_[BSP_FDCAN_SINGLE_MAX_DEVICE_NUM];
    uint8_t rx_route_count_;
    uint8_t filter_configured_route_count_; // 已配置过滤器的route数
    bool is_started_; // 表示FDCAN是否已经启动

    bspFDCANErrorCallback_f error_callback_;
} bspFDCANInstance_t;

static uint8_t fdcan_memory_index_ = 0;
static bspFDCANInstance_t fdcan_instance_memory_[BSP_FDCAN_MAX_INSTANCE_NUM] = {0};

static bspFDCANStatus_e bspFDCANGetStatusFromHAL(HAL_StatusTypeDef status_hal)
{
    switch (status_hal) {
        case HAL_OK:
            return BSP_FDCAN_OK;
        case HAL_ERROR:
            return BSP_FDCAN_ERROR;
        case HAL_BUSY:
            return BSP_FDCAN_BUSY;
        case HAL_TIMEOUT:
            return BSP_FDCAN_TIMEOUT;
        default:
            break;
    }
    return BSP_FDCAN_ERROR;
}

static bspFDCANIdType_e HALIdTypeToBspFDCANIdType(uint32_t id_type_hal)
{
    switch (id_type_hal) {
        case FDCAN_STANDARD_ID:
            return BSP_FDCAN_ID_STD;
        case FDCAN_EXTENDED_ID:
            return BSP_FDCAN_ID_EXT;
        default:
            break;
    }

    return BSP_FDCAN_ID_EXT;
}

static uint32_t bspFDCANIdTypeToHALIdType(bspFDCANIdType_e id_type_bsp)
{
    switch (id_type_bsp) {
        case BSP_FDCAN_ID_STD:
            return FDCAN_STANDARD_ID;
        case BSP_FDCAN_ID_EXT:
            return FDCAN_EXTENDED_ID;
        default:
            break;
    }

    return FDCAN_STANDARD_ID;
}

static uint32_t bspFDCANFrameTypeToHAL(bspFDCANFrameType_e frame_type_bsp)
{
    switch (frame_type_bsp) {
        case BSP_FDCAN_DATA_FRAME:
            return FDCAN_DATA_FRAME;
        case BSP_FDCAN_REMOTE_FRAME:
            return FDCAN_REMOTE_FRAME;
        default:
            break;
    }

    return FDCAN_DATA_FRAME;
}

static uint32_t lengthToHALDlc(uint8_t len)
{
    if (len <= 0U)  return FDCAN_DLC_BYTES_0;
    if (len <= 1U)  return FDCAN_DLC_BYTES_1;
    if (len <= 2U)  return FDCAN_DLC_BYTES_2;
    if (len <= 3U)  return FDCAN_DLC_BYTES_3;
    if (len <= 4U)  return FDCAN_DLC_BYTES_4;
    if (len <= 5U)  return FDCAN_DLC_BYTES_5;
    if (len <= 6U)  return FDCAN_DLC_BYTES_6;
    if (len <= 7U)  return FDCAN_DLC_BYTES_7;
    if (len <= 8U)  return FDCAN_DLC_BYTES_8;
    if (len <= 12U) return FDCAN_DLC_BYTES_12;
    if (len <= 16U) return FDCAN_DLC_BYTES_16;
    if (len <= 20U) return FDCAN_DLC_BYTES_20;
    if (len <= 24U) return FDCAN_DLC_BYTES_24;
    if (len <= 32U) return FDCAN_DLC_BYTES_32;
    if (len <= 48U) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

static uint8_t HALDlcToLength(uint32_t hal_dlc)
{
    switch (hal_dlc) {
        case FDCAN_DLC_BYTES_0:  return 0U;
        case FDCAN_DLC_BYTES_1:  return 1U;
        case FDCAN_DLC_BYTES_2:  return 2U;
        case FDCAN_DLC_BYTES_3:  return 3U;
        case FDCAN_DLC_BYTES_4:  return 4U;
        case FDCAN_DLC_BYTES_5:  return 5U;
        case FDCAN_DLC_BYTES_6:  return 6U;
        case FDCAN_DLC_BYTES_7:  return 7U;
        case FDCAN_DLC_BYTES_8:  return 8U;
        case FDCAN_DLC_BYTES_12: return 12U;
        case FDCAN_DLC_BYTES_16: return 16U;
        case FDCAN_DLC_BYTES_20: return 20U;
        case FDCAN_DLC_BYTES_24: return 24U;
        case FDCAN_DLC_BYTES_32: return 32U;
        case FDCAN_DLC_BYTES_48: return 48U;
        case FDCAN_DLC_BYTES_64: return 64U;
        default: 
            return 0U;
    }
}

bool bspFDCANDataLengthIsValid(uint8_t data_length)
{
    if (data_length <= 8U) {
        return true;
    }

    switch (data_length) {
        case 12U:
        case 16U:
        case 20U:
        case 24U:
        case 32U:
        case 48U:
        case 64U:
            return true;
        default:
            return false;
    }
}

static bspFDCANInstance_t *bspFDCANFindInstanceByHandle(FDCAN_HandleTypeDef *fdcan_handle)
{
    for (size_t i = 0; i < fdcan_memory_index_; i ++) {
        if (fdcan_instance_memory_[i].fdcan_handle_ == fdcan_handle) {
            return &fdcan_instance_memory_[i];
        }
    }
    return NULL;
}

static bspFDCANRxRoute_t *bspFDCANFindRxRouteByHeader(bspFDCANInstance_t *instance,
                                                    const FDCAN_RxHeaderTypeDef *rx_message_header_hal)
{
    if (instance == NULL || rx_message_header_hal == NULL) {
        return NULL;
    }

    uint32_t rx_message_id = rx_message_header_hal->Identifier;

    for (size_t i = 0; i < instance->rx_route_count_; i ++) {
        bspFDCANRxRoute_t *rx_route = &instance->rx_route_[i];
        if (rx_route->route_ide_ != HALIdTypeToBspFDCANIdType(rx_message_header_hal->IdType)) {
            continue;
        }

        if (rx_route->route_id_mask_ == 0U) {
            // 精确匹配
            if (rx_route->route_id_ == rx_message_id) {
                return rx_route;
            }
        } else {
            // 范围匹配
            if ((rx_route->route_id_ & rx_route->route_id_mask_) == (rx_message_id & rx_route->route_id_mask_)) {
                return rx_route;
            }
        }
    }

    return NULL;
}

static void bspFDCANPackRxMessage(const FDCAN_RxHeaderTypeDef *rx_message_header_hal,
                                    const uint8_t *rx_message_data,
                                    bspFDCANMessage_t *rx_message)
{
    if (rx_message_header_hal == NULL || rx_message_data == NULL || rx_message == NULL) {
        return;
    }

    memset(rx_message, 0, sizeof(bspFDCANMessage_t));

    rx_message->message_header_.message_id_ = rx_message_header_hal->Identifier;
    rx_message->message_header_.message_ide_ = (rx_message_header_hal->IdType == FDCAN_STANDARD_ID) ? BSP_FDCAN_ID_STD : BSP_FDCAN_ID_EXT;
    rx_message->message_header_.message_rtr_ = (rx_message_header_hal->RxFrameType == FDCAN_REMOTE_FRAME) ? BSP_FDCAN_REMOTE_FRAME : BSP_FDCAN_DATA_FRAME;
    rx_message->message_header_.message_length_ = HALDlcToLength(rx_message_header_hal->DataLength);

    if (rx_message_header_hal->FDFormat == FDCAN_CLASSIC_CAN) {
        rx_message->message_header_.message_format_ = BSP_FDCAN_FRAME_CLASSIC;
    } else if (rx_message_header_hal->BitRateSwitch == FDCAN_BRS_ON) {
        rx_message->message_header_.message_format_ = BSP_FDCAN_FRAME_FD_BRS;
    } else {
        rx_message->message_header_.message_format_ = BSP_FDCAN_FRAME_FD_NO_BRS;
    }

    rx_message->message_header_.message_brs_ = (rx_message_header_hal->BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;
    rx_message->message_header_.message_esi_ = (rx_message_header_hal->ErrorStateIndicator == FDCAN_ESI_PASSIVE) ? 1U : 0U;

    memcpy(rx_message->message_data_,
           rx_message_data,
           rx_message->message_header_.message_length_);
}

bspFDCANInstance_t *bspFDCANInit(const bspFDCANConfig_t *config)
{
    if (config == NULL) {
        return  NULL;
    }

    if (fdcan_memory_index_ >= BSP_FDCAN_MAX_INSTANCE_NUM) {
        return NULL;
    }

    bspFDCANInstance_t *instance = &fdcan_instance_memory_[fdcan_memory_index_];
    memset(instance, 0, sizeof(bspFDCANInstance_t));
    instance->fdcan_handle_ = config->fdcan_handle_;
    instance->name_ = config->name_;
    instance->rx_route_count_ = 0;
    instance->filter_configured_route_count_ = 0;
    instance->is_started_ = false;

    fdcan_memory_index_ ++;

    return instance;
}

void bspFDCANRxCallbackRegister(bspFDCANInstance_t *instance, bspFDCANRxRoute_t rx_route)
{
    if (instance == NULL || 
        rx_route.route_owner_ == NULL || 
        rx_route.route_rx_callback_ == NULL ||
        instance->rx_route_count_ >= BSP_FDCAN_SINGLE_MAX_DEVICE_NUM) {
        return;
    }

    instance->rx_route_[instance->rx_route_count_] = rx_route;
    instance->rx_route_count_ ++;
}

void bspFDCANErrorCallbackRegister(bspFDCANInstance_t *instance, bspFDCANErrorCallback_f callback)
{
    instance->error_callback_ = callback;
}

// 暂时把过滤器配置写死
// 列表模式，只考虑标准帧
bspFDCANStatus_e bspFDCANSetFilter(bspFDCANInstance_t *instance)
{   
    if (instance == NULL || instance->fdcan_handle_ == NULL || instance->rx_route_count_ == 0) {
        return BSP_FDCAN_ERROR;
    }

    if (instance->filter_configured_route_count_ == instance->rx_route_count_) {
        return BSP_FDCAN_OK;
    }

    if (instance->is_started_ == true) {
        return BSP_FDCAN_BUSY;
    }

    HAL_StatusTypeDef status_hal;

    FDCAN_FilterTypeDef filter_config = {0};
    for (size_t i = 0; i < instance->rx_route_count_; i ++) {
        filter_config.IdType = bspFDCANIdTypeToHALIdType(instance->rx_route_[i].route_ide_);
        filter_config.FilterIndex = i; // 浪费的写法
        filter_config.FilterType = FDCAN_FILTER_RANGE;
        filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 简单实现，全部进FIFO0
        filter_config.FilterID1 = instance->rx_route_[i].route_id_;
        filter_config.FilterID2 = instance->rx_route_[i].route_id_; // 范围匹配，现在这样表示只匹配一个ID

        status_hal = HAL_FDCAN_ConfigFilter(instance->fdcan_handle_, &filter_config);
        if (status_hal != HAL_OK) {
            return bspFDCANGetStatusFromHAL(status_hal);
        }
    }

    instance->filter_configured_route_count_ = instance->rx_route_count_;

    return BSP_FDCAN_OK;
}

bspFDCANStatus_e bspFDCANStart(bspFDCANInstance_t *instance)
{
    if (instance == NULL || instance->fdcan_handle_ == NULL) {
        return BSP_FDCAN_ERROR;
    }

    if (instance->is_started_ == true) {
        return BSP_FDCAN_OK;
    }

    HAL_StatusTypeDef status_hal;

    // 启动CAN外设
    status_hal = HAL_FDCAN_Start(instance->fdcan_handle_);
    if (status_hal != HAL_OK) {
        return bspFDCANGetStatusFromHAL(status_hal);
    }
    // 开启FIFO0新消息中断，只要FIFO0有至少一个报文，就触发中断
    // FDCAN_TX_BUFFER0会被忽略
    status_hal = HAL_FDCAN_ActivateNotification(instance->fdcan_handle_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);
    if (status_hal != HAL_OK) {
        return bspFDCANGetStatusFromHAL(status_hal);
    }
    // 开启FIFO1新消息中断，只要FIFO1有至少一个报文，就触发中断
    // FDCAN_TX_BUFFER0会被忽略
    status_hal = HAL_FDCAN_ActivateNotification(instance->fdcan_handle_, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_TX_BUFFER0);
    if (status_hal != HAL_OK) {
        return bspFDCANGetStatusFromHAL(status_hal);
    }

    instance->is_started_ = true;

    return BSP_FDCAN_OK;
}

// 无阻塞，之后可能可以加一个阻塞等待，但是这个等待给上层处理或许更好？
bspFDCANStatus_e bspFDCANTransmit(bspFDCANInstance_t *instance, const bspFDCANMessage_t *tx_message)
{
    if (instance == NULL || 
        instance->fdcan_handle_ == NULL || 
        tx_message == NULL || 
        tx_message->message_header_.message_length_ > BSP_FDCAN_MAX_DATA_LENGTH) { // 注意载荷为0没关系
        return BSP_FDCAN_ERROR;
    }

    // 不接受需要补0的数据长度
    if (bspFDCANDataLengthIsValid(tx_message->message_header_.message_length_) == false) {
        return BSP_FDCAN_ERROR;
    }

    if (tx_message->message_header_.message_format_ != BSP_FDCAN_FRAME_CLASSIC && 
        tx_message->message_header_.message_rtr_ == BSP_FDCAN_REMOTE_FRAME) {
        // FDCAN不支持遥控帧
        return BSP_FDCAN_ERROR;
    }

    // classical不允许dlc超过8
    if (tx_message->message_header_.message_format_ == BSP_FDCAN_FRAME_CLASSIC &&
        tx_message->message_header_.message_length_ > 8U) {
        return BSP_FDCAN_ERROR;
    }

    HAL_StatusTypeDef status_hal;
    // 检查tx FIFO是否空闲
    if (HAL_FDCAN_GetTxFifoFreeLevel(instance->fdcan_handle_) == 0) {
        // 没有空位
        return BSP_FDCAN_BUSY;
    }
    
    FDCAN_TxHeaderTypeDef tx_message_header_hal = {
        .Identifier = tx_message->message_header_.message_id_,
        .IdType = (tx_message->message_header_.message_ide_ == BSP_FDCAN_ID_STD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID,
        .TxFrameType = (tx_message->message_header_.message_rtr_ == BSP_FDCAN_DATA_FRAME) ? FDCAN_DATA_FRAME : FDCAN_REMOTE_FRAME,
        .DataLength = lengthToHALDlc(tx_message->message_header_.message_length_),
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = (tx_message->message_header_.message_format_ == BSP_FDCAN_FRAME_FD_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
        .FDFormat = (tx_message->message_header_.message_format_ == BSP_FDCAN_FRAME_CLASSIC) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0U,
    };

    // 添加到FIFO或者Queue，按照cubemx中配置进行
    // Queue按照ID优先级，FIFO按照先后顺序
    status_hal = HAL_FDCAN_AddMessageToTxFifoQ(instance->fdcan_handle_, &tx_message_header_hal, tx_message->message_data_);
    
    return bspFDCANGetStatusFromHAL(status_hal);
}

uint8_t bspFDCANGetValidDataLengthCeil(uint8_t data_length)
{
    if (data_length <= 8U) {
        return data_length;
    }

    if (data_length <= 12U) {
        return 12U;
    }

    if (data_length <= 16U) {
        return 16U;
    }

    if (data_length <= 20U) {
        return 20U;
    }

    if (data_length <= 24U) {
        return 24U;
    }

    if (data_length <= 32U) {
        return 32U;
    }

    if (data_length <= 48U) {
        return 48U;
    }

    if (data_length <= 64U) {
        return 64U;
    }

    return 0U;
}

static void bspFDCANFifoxNewMsgCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RX_FIFOx)
{
    bspFDCANInstance_t *instance = bspFDCANFindInstanceByHandle(hfdcan);
    if (instance == NULL) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_message_header_hal = {0};
    uint8_t rx_message_data[BSP_FDCAN_MAX_DATA_LENGTH] = {0};
    bspFDCANMessage_t rx_message = {0};
    bspFDCANRxRoute_t *rx_route = NULL;
    
    // 循环，把fifox中的报文全部处理完
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, RX_FIFOx)) {
        // 必须从FIFO中取走消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, RX_FIFOx, &rx_message_header_hal, rx_message_data) != HAL_OK) {
            return;
        }
        bspFDCANPackRxMessage(&rx_message_header_hal, rx_message_data, &rx_message);

        // 按照报文ID调用对应的回调函数
        rx_route = bspFDCANFindRxRouteByHeader(instance, &rx_message_header_hal);
        if (rx_route != NULL && rx_route->route_rx_callback_ != NULL) {
            rx_route->route_rx_callback_(rx_route->route_owner_, &rx_message);
        }
    } 
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{   
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        bspFDCANFifoxNewMsgCallback(hfdcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{   
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
        bspFDCANFifoxNewMsgCallback(hfdcan, FDCAN_RX_FIFO1);
    }
}

