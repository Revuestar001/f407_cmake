#pragma once

typedef enum
{
    APP_STATE_UNINIT = 0,
    APP_STATE_CALIBRATING,
    APP_STATE_NORMAL,
    APP_STATE_DEGRADED,
    APP_STATE_REINIT,
} appState_e;