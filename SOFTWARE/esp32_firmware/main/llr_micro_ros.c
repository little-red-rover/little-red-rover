#include <rcl/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_mgr.h"

#include "micro_ros_mgr.h"

void app_main(void)
{
	wifi_mgr_init();

	micro_ros_mgr_init();
}