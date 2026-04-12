#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <math.h>
#include "general_math.h"
#include "1st_order_lp_filter.h"

static bool computeCutOffFrequency(filterOneOrderLPF_t *instance)
{   
    if (instance == NULL) {
        return false;
    }

    float cut_off_freq_hz;
    cut_off_freq_hz = logf(instance->filter_coefficient_) * instance->sample_frequency_hz_ / (-2.0f * USER_PI);
    instance->cut_off_frequency_hz_ = cut_off_freq_hz;

    return true;
}

static bool computeK(filterOneOrderLPF_t *instance)
{   
    if (instance == NULL) {
        return false;
    }

    float k;
    k = expf((-2.0f * USER_PI) * instance->cut_off_frequency_hz_ / instance->sample_frequency_hz_);
    instance->filter_coefficient_ = k;

    return true;
}

bool filterOneOrderLPFInitByK(filterOneOrderLPF_t *instance, float k, float dt_s, float init_value, filterOneOrderLPFMode_e mode, filterOneOrderLPFGetAbsTimeUs_f callback)
{
    if (instance == NULL || k <= 0.0f || k > 1.0f || dt_s <= 0.0f) {
        return false;
    }

    // 没有注册获取时间戳的函数，不可以使用可变dt模式
    if (callback == NULL) {
        mode = FILTER_1ST_ORDER_LPF_MODE_FIXED_DT;
    }

    memset(instance, 0, sizeof(filterOneOrderLPF_t));
    instance->filter_coefficient_ = k;
    instance->dt_s_ = dt_s;
    instance->sample_frequency_hz_ = 1.0f / dt_s;
    instance->mode_ = mode;
    if (computeCutOffFrequency(instance) == false) {
        return false;
    }
    instance->filtered_value_ = init_value;
    instance->get_time_us_callback_ = callback;
    if (instance->get_time_us_callback_ != NULL) {
        instance->timestamp_last_us_ = instance->get_time_us_callback_();
    }
    instance->is_initialized_ = true;

    return true;
}

bool filterOneOrderLPFInitByCutOffFreq(filterOneOrderLPF_t *instance, float fc, float dt_s, float init_value, filterOneOrderLPFMode_e mode, filterOneOrderLPFGetAbsTimeUs_f callback)
{
    if (instance == NULL || fc < 0.0f || dt_s <= 0.0f) {
        return false;
    }

    float sample_freq = 1.0f / dt_s;
    if (fc >= sample_freq * 0.5f) {
        // 截止频率超出范围，截止频率意义不正确
        return false;
    }

    // 没有注册获取时间戳的函数，不可以使用可变dt模式
    if (callback == NULL) {
        mode = FILTER_1ST_ORDER_LPF_MODE_FIXED_DT;
    }

    memset(instance, 0, sizeof(filterOneOrderLPF_t));
    instance->cut_off_frequency_hz_ = fc;
    instance->dt_s_ = dt_s;
    instance->sample_frequency_hz_ = 1.0f / dt_s;
    instance->mode_ = mode;
    if (computeK(instance) == false) {
        return false;
    }
    instance->filtered_value_ = init_value;
    instance->get_time_us_callback_ = callback;
    if (instance->get_time_us_callback_ != NULL) {
        instance->timestamp_last_us_ = instance->get_time_us_callback_();
    }
    instance->is_initialized_ = true;

    return true;
}

bool filterOneOrderLPFUpdate(filterOneOrderLPF_t *instance, float new_value, float *filtered)
{
    if (instance == NULL || instance->is_initialized_ == false || filtered == NULL) {
        return false;
    }

    if (instance->mode_ == FILTER_1ST_ORDER_LPF_MODE_VARIABLE_DT) {
        uint64_t timestamp_us = instance->get_time_us_callback_();
        float dt_s = (float)(timestamp_us - instance->timestamp_last_us_) * 1.0e-6f;
        if (dt_s <= 0.0f) {
            return false;
        }
        instance->dt_s_ = dt_s;
        instance->sample_frequency_hz_ = 1.0f / dt_s;
        if (computeK(instance) == false) {
            return false;
        }
    }

    instance->filtered_value_ = (1.0f - instance->filter_coefficient_) * new_value + instance->filter_coefficient_ * instance->filtered_value_;
    *filtered = instance->filtered_value_;

    return true;
}