// excerpts from:
// https://github.com/nopnop2002/esp-idf-lsm6ds3/blob/main/components/I2Cdev/I2Cdev.cpp

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "hal/i2c_types.h"

#define I2C_NUM I2C_NUM_0

bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return true;
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param length Number of bytes to write
 * @param data Array of bytes to write
 * @return Status of operation (true = success)
 */
bool writeBytes(uint8_t devAddr,
				uint8_t regAddr,
				uint8_t length,
				uint8_t *data,
				void *wireObj)
{
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
	ESP_ERROR_CHECK(i2c_master_write(cmd, data, length - 1, 0));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length - 1], 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
	return true;
}

void SelectRegister(uint8_t dev, uint8_t reg)
{
	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, 1));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, 1));
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off
 * to use default class value in I2Cdev::readTimeout)
 * @return I2C_TransferReturn_TypeDef
 * http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
int8_t readBytes(uint8_t devAddr,
				 uint8_t regAddr,
				 uint8_t length,
				 uint8_t *data)
{
	i2c_cmd_handle_t cmd;
	SelectRegister(devAddr, regAddr);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, 1));

	if (length > 1)
		ESP_ERROR_CHECK(i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK));

	ESP_ERROR_CHECK(
	  i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK));

	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	ESP_ERROR_CHECK(
	  i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
	i2c_cmd_link_delete(cmd);

	return length;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off
 * to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
	return readBytes(devAddr, regAddr, 1, data);
}
