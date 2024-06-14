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
#include "pub_sub_utils.h"

#include "drive_base_driver.h"
#include "lidar_driver.h"

#include "driver/gpio.h"
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>

void app_main(void)
{
	// Set GPIO so motors don't immediately start running.
	// This won't be needed in the next hardware revision.
	gpio_set_direction(7, GPIO_MODE_OUTPUT);
	gpio_set_direction(8, GPIO_MODE_OUTPUT);
	gpio_set_direction(9, GPIO_MODE_OUTPUT);
	gpio_set_direction(10, GPIO_MODE_OUTPUT);

	gpio_set_level(7, 0);
	gpio_set_level(8, 0);
	gpio_set_level(9, 0);
	gpio_set_level(10, 0);

	wifi_mgr_init();

	lidar_driver_init();

	drive_base_driver_init();

	micro_ros_mgr_init();
}