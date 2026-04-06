#pragma once
#include <stdint.h>

void bspDWTInit();

uint32_t bspDWTGetCount(void);
// 84MHz下最大51s
uint32_t bspDWTGetElapsedTimeUs(uint32_t start_cnt);
// 84MHz下最大51s
void bspDWTDelayUs(uint32_t time_us);
void bspDWTDelayMs(uint32_t time_ms);
// 获取绝对us时间戳，从bspDWTInit开始，注意，任意两次调用需要保证不超过一个溢出的时间，从bspDWTInit开始的第一次调用也要保证
uint64_t bspDWTGetAbsTimeUs(void);