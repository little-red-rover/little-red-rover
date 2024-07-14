#include <rcl/types.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_mgr.h"

#include "micro_ros_mgr.h"

#include "LSM6DS3_imu_driver.h"
#include "drive_base_driver.h"
#include "lidar_driver.h"

#include "driver/gpio.h"
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/int32.h>

void app_main(void)
{
	drive_base_driver_init();

	LSM6DS3_imu_driver_init();

	lidar_driver_init();

	wifi_mgr_init();

	micro_ros_mgr_init();
}
