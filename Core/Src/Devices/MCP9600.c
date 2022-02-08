/* *
 *  Created on: 17 Jun 2020
 *  Author: ARIS / Linus Stoeckli
 */


#include <devices/MCP9600.h>
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>


int mcp9600_init(struct mcp9600_dev * dev)
{
	if (HAL_I2C_GetState(dev->i2c_bus) != HAL_I2C_STATE_READY)
	{
		printf("i2c1 not ready!\n");
	} else {
		printf("i2c1 is ready!\n");
	}

	HAL_StatusTypeDef _ret;

	_ret = HAL_I2C_IsDeviceReady(dev->i2c_bus, dev->addr, 10, dev->delay);
	if ( _ret != HAL_OK )
	{
		printf("MCP9600 setup fail\n");
		printf("Errorcode: %d\n", _ret);
		return 0;
	}

	printf("MCP9600 setup success\n");
	return 1;
}

void mcp9600_read(struct mcp9600_dev * dev, float * dat)
{
	uint8_t T_H[1] = {0x00};
	uint8_t T_D[1] = {0x01};
	uint8_t T_C[1] = {0x02};

	float temp_H;
	float temp_D;
	float temp_C;
	int lowTemp;
	uint8_t rec[2];



	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, T_H, 1, dev->delay);
	HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, rec, 2, dev->delay);

	lowTemp = rec[0] & 0x80;

	if (lowTemp) {
		temp_H = (float)rec[0] * 16 + (float)rec[1] / 16 - 4096;
	} else {
		temp_H = (float)rec[0] * 16 + (float)rec[1] / 16;
	}

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, T_D, 1, dev->delay);
	HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, rec, 2, dev->delay);

	lowTemp = rec[0] & 0x80;

	if (lowTemp) {
		temp_D = (float)rec[0] * 16 + (float)rec[1] / 16 - 4096;
	} else {
		temp_D = (float)rec[0] * 16 + (float)rec[1] / 16;
	}

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, T_C, 1, dev->delay);
	HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, rec, 2, dev->delay);

	lowTemp = rec[0] & 0x80;

	if (lowTemp) {
		temp_C = (float)rec[0] * 16 + (float)rec[1] / 16 - 4096;
	} else {
		temp_C = (float)rec[0] * 16 + (float)rec[1] / 16;
	}

	dat[0] = temp_H;
	dat[1] = temp_D;
	dat[2] = temp_C;

};

void write(struct mcp9600_dev * dev, uint8_t REG, uint8_t val)
{
	uint8_t buf[2];
	buf[0] = REG;
	buf[1] = val;

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 2, dev->delay);

};

