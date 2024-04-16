#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "wifi_mgr.h"

void app_main(void)
{
    wifi_mgr_init();
}