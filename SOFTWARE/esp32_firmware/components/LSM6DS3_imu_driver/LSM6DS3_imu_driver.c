// https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf

#include "LSM6DS3_imu_driver.h"
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pub_sub_utils.h"
#include "sdkconfig.h"
#include "sensor_msgs/msg/imu.h"
#include <stdio.h>
#include <time.h>

#include "micro_ros_mgr.h"
#include <rcl/rcl.h>

#include <math.h>

#define IMU_TASK_STACK_SIZE (2048)

#define SCL_PIN 2
#define SDA_PIN 3

rcl_publisher_t *imu_publisher;
i2c_master_dev_handle_t imu_i2c_handle;

static void imu_driver_task(void *arg)
{
	while (get_uros_state() != AGENT_CONNECTED) {
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
	while (1) {
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void LSM6DS3_imu_driver_init()
{
	i2c_master_bus_config_t i2c_bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = -1,
		.scl_io_num = SCL_PIN,
		.sda_io_num = SDA_PIN,
		.glitch_ignore_cnt = 7,
	};
	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

	i2c_device_config_t imu_i2c_conf = { .scl_speed_hz = 400000,
										 .device_address = 0b1101011 };

	ESP_ERROR_CHECK(
	  i2c_master_bus_add_device(bus_handle, &imu_i2c_conf, &imu_i2c_handle));

	i2c_master_transmit_receive();

	imu_publisher = register_publisher(
	  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, imu), "lrr_imu");

	xTaskCreate(
	  imu_driver_task, "imu_driver_task", IMU_TASK_STACK_SIZE, NULL, 10, NULL);
}