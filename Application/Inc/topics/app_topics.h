#pragma once

#include "topic_bus.h"

#define MSG_RC_COMMAND_TOPIC_DEPTH 1U
#define MSG_RC_COMMAND_TOPIC_MAX_WAITER 8U
#define MSG_RC_COMMAND_TOPIC_NAME "/rc_command"

#define MSG_INS_TOPIC_DEPTH 1U
#define MSG_INS_TOPIC_MAX_WAITER 8U
#define MSG_INS_TOPIC_NAME "/ins"

typedef enum
{
    APP_TOPICS_RC_COMMAND = 0,
    APP_TOPICS_INS,
    APP_TOPICS_MAX,
} appTopics_e;

moduleTopic_t *appTopicsGet(appTopics_e topic_id);