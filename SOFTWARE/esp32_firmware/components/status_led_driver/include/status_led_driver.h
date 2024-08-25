#pragma once

enum eStatus
{
    eWifiDisconnected,
    eWifiProvisioning,
    eWifiConnected,
    eAgentDisconnected,
    eAgentConnected,
    eSystemError,
    eImuInitFailed,
    eLidarInitFailed,
    eDriveBaseInitFailed,
    eSystemGood
};

void set_status(enum eStatus status);

void status_led_driver_init();
