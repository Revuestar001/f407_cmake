#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_def.h"

#define APP_LOG_TRY_INIT_DELAY_MS 100U

void appLogTaskEntry(void *argument);