// https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf

#include "LSM6DS3_imu_driver.h"

#include <stdint.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pub_sub_utils.h"
#include "sensor_msgs/msg/imu.h"
#include <stdio.h>
#include <time.h>

#include "micro_ros_mgr.h"
#include <rcl/rcl.h>

#define IMU_TASK_STACK_SIZE (2048)

#define SCL_PIN 2
#define SDA_PIN 3

static const char *TAG = "imu driver";

rcl_publisher_t *imu_publisher;
i2c_master_dev_handle_t imu_i2c_handle;

void readRegisters(uint8_t address, uint8_t *data, size_t length)
{
	ESP_ERROR_CHECK(i2c_master_transmit_receive(
	  imu_i2c_handle, &address, 1, data, length, -1));
}

uint8_t readRegister(uint8_t address)
{
	uint8_t read;
	readRegisters(address, &read, 1);
	return read;
}

void writeRegister(uint8_t address, uint8_t value)
{
	uint8_t buff[] = { address, value };

	ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle, buff, 2, -1));
}

void writeRegisters(uint8_t address, uint8_t *values, size_t length)
{
	uint8_t buff[] = { address };

	ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle, buff, 2, -1));
	ESP_ERROR_CHECK(i2c_master_transmit(imu_i2c_handle, values, length, -1));
}

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
		.i2c_port = I2C_NUM_0,
		.scl_io_num = SCL_PIN,
		.sda_io_num = SDA_PIN,
		.glitch_ignore_cnt = 7,
	};
	i2c_master_bus_handle_t bus_handle;

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

	i2c_device_config_t imu_i2c_conf = { .scl_speed_hz = 400000,
										 .device_address = LSM6DS3_ADDRESS };

	ESP_ERROR_CHECK(
	  i2c_master_bus_add_device(bus_handle, &imu_i2c_conf, &imu_i2c_handle));

	// i2c_master_transmit_receive();

	// imu_publisher = register_publisher(
	//   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, imu), "lrr_imu");
	// uint8_t write_addr = (0x6A << 1) | 1;
	// uint8_t read;
	// i2c_master_transmit_receive(
	//   imu_i2c_handle, &DEV_READ_ADDR, 1, &read, 1, -1);

	ESP_LOGI(TAG, "IMU WHO_AM_I: %d", (int)readRegister(LSM6DS3_WHO_AM_I_REG));

	xTaskCreate(
	  imu_driver_task, "imu_driver_task", IMU_TASK_STACK_SIZE, NULL, 10, NULL);
}
