#pragma once

#include <stdint.h>

typedef uint32_t bspCriticalIRQState_t;

bspCriticalIRQState_t bspCriticalEnter(void);
void bspCriticalExit(bspCriticalIRQState_t primask);