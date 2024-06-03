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

#include "lidar_driver.h"

#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>

void app_main(void)
{
	wifi_mgr_init();

	// Publishers must be registered BEFORE calling micro_ros_mgr_init
	rcl_publisher_t *lidar_publisher = register_publisher(
	  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
	  "lrr_lidar_scan");

	micro_ros_mgr_init();

	lidar_driver_init(lidar_publisher);
}