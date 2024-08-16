#pragma once

typedef enum RX_MSG_TYPES
{
    eTwistCmd
} eRxMsgTypes;

void register_callback(void (*callback)(void *), eRxMsgTypes type);

void socket_mgr_init();
