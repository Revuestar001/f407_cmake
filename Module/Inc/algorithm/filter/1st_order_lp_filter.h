#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FILTER_1ST_ORDER_LPF_MODE_FIXED_DT = 0, // 固定dt,此模式下采样频率固定
    FILTER_1ST_ORDER_LPF_MODE_VARIABLE_DT, // 可变dt，此模式下采样频率可变，会重新计算k
} filterOneOrderLPFMode_e;

typedef uint64_t (*filterOneOrderLPFGetAbsTimeUs_f)(void);

typedef struct one_order_lp_filter
{
    float filter_coefficient_;
    float cut_off_frequency_hz_;
    float sample_frequency_hz_;

    float filtered_value_;
    
    filterOneOrderLPFGetAbsTimeUs_f get_time_us_callback_;
    uint64_t timestamp_last_us_;

    float dt_s_;

    filterOneOrderLPFMode_e mode_; // 不允许非初始化时更改模式！
    bool is_initialized_;
} filterOneOrderLPF_t;

// 使用k初始化
bool filterOneOrderLPFInitByK(filterOneOrderLPF_t *instance, float k, float dt_s, float init_value, filterOneOrderLPFMode_e mode, filterOneOrderLPFGetAbsTimeUs_f callback);
// 使用截止频率初始化
bool filterOneOrderLPFInitByCutOffFreq(filterOneOrderLPF_t *instance, float fc, float dt_s, float init_value, filterOneOrderLPFMode_e mode, filterOneOrderLPFGetAbsTimeUs_f callback);
// 更新并输出滤波后的值，fixed_dt模式下，使用初始化时传入的固定dt进行(其实计算的时候和dt无关)，可变dt模式下，根据时间戳计算dt并自动计算新的k值
bool filterOneOrderLPFUpdate(filterOneOrderLPF_t *instance, float new_value, float *filtered);