#pragma once

void bspDWTInit();

uint32_t bspDWTGetCount(void);
// 84MHz下最大51s
uint32_t bspDWTGetElapsedTimeUs(uint32_t start_cnt);
// 84MHz下最大51s
void bspDWTDelayUs(uint32_t time_us);
void bspDWTDelayMs(uint32_t time_ms);