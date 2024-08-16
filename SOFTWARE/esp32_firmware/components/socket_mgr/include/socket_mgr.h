#pragma once

#include "freertos/idf_additions.h"

typedef enum RX_MSG_TYPES
{
    eTwistCmd
} eRxMsgTypes;

extern QueueHandle_t tx_queue;

void register_callback(void (*callback)(void *), eRxMsgTypes type);

void socket_mgr_init();
