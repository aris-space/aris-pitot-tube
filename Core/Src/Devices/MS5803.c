/* *
 *  Created on: 29 Aug 2020
 *  Author: ARIS / Linus Stoeckli
 */

#include <devices/MS5803.h>
#include <stdio.h>
#include <math.h>

uint8_t ms5803_init(struct ms5803_dev *dev) {
	if (HAL_I2C_GetState(dev->i2c_bus) != HAL_I2C_STATE_READY) {
		printf("i2c not ready!\n");
		return 0;
	} else {
		printf("i2c is ready!\n");
	}
	HAL_StatusTypeDef _ret;
	_ret = HAL_I2C_IsDeviceReady(dev->i2c_bus, dev->addr, 10, dev->addr);
	if (_ret != HAL_OK) {
		printf("BARO setup fail\n");
		printf("Errorcode: %d\n", _ret);
		return 0;
	}

	//get factory calibration data
	//reset (advised in datasheet)

	uint8_t reset_code[1];
	reset_code[0] = 0x1E;
	_ret = HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, reset_code, 1,
			dev->delay);

	HAL_Delay(100);

	//6 calibration values with each having 2 bytes

	//get each calibration value (c1 - c6 in datasheet)
	uint8_t get_add;
	uint8_t buf[2];

	for (int i = 1; i < 7; i++) {

		//standard commands (see datasheet)
		get_add = 0b10100000;
		get_add = get_add + 2 * i;

		HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, &get_add, 1,
				dev->delay);
		HAL_Delay(15);
		_ret = HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, buf, 2,
				dev->delay);
		dev->cal[i - 1] = (uint16_t) (buf[0] << 8) | buf[1];

		if (_ret != HAL_OK) {
			printf("MS5607 cal read fail\n");
			return 0;
		}
	}

	printf("BARO setup success\n");

	buf[0] = 0x44;
	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	osDelay(3);
	// need to wait 3 ms
	return 1;
}

uint8_t ms5803_prep_pressure(struct ms5803_dev *dev) {
	uint8_t buf[3];
	uint8_t res;
	buf[0] = 0x00;

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	res = HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, buf, 3, dev->delay);

	dev->D1 = (uint32_t) (buf[0] << 16) | (uint32_t) (buf[1] << 8)
			| (uint32_t) buf[2];

	buf[0] = 0x54;
	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	return res;
	// need to wait 3 ms
}

uint8_t ms5803_read_pressure(struct ms5803_dev *dev) {
	uint8_t buf[3];
	uint8_t res;
	buf[0] = 0x00;

	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	res = HAL_I2C_Master_Receive(dev->i2c_bus, dev->addr, buf, 3, dev->delay);

	dev->D2 = (uint32_t) (buf[0] << 16) | (uint32_t) (buf[1] << 8)
			| (uint32_t) buf[2];

	buf[0] = 0x44;
	HAL_I2C_Master_Transmit(dev->i2c_bus, dev->addr, buf, 1, dev->delay);
	// need to wait 3 ms
	return res;
}

void ms5803_convert(struct ms5803_dev *dev, float *p, float *t) {
	//calculate calibration values
	uint16_t c1 = dev->cal[0];
	uint16_t c2 = dev->cal[1];
	uint16_t c3 = dev->cal[2];
	uint16_t c4 = dev->cal[3];
	uint16_t c5 = dev->cal[4];
	uint16_t c6 = dev->cal[5];

	uint32_t D1 = dev->D1;
	uint32_t D2 = dev->D2;

	//calculations from datasheet
	int32_t dT = 0, TEMP = 0, T2 = 0, P = 0;
	int64_t OFF2 = 0, SENS2 = 0, OFF = 0, SENS = 0;

	dT = D2 - ((int32_t) c5 << 8);
	TEMP = 2000 + ((int64_t) dT * c6 >> 23);

	if (TEMP < 2000) {
		T2 = 3 * (((int64_t) dT * dT) >> 33);
		OFF2 = 3 * (TEMP - 2000.0) * (TEMP - 2000.0) / 2;
		SENS2 = 5 * (TEMP - 2000.0) * (TEMP - 2000.0) / 8;
		if (TEMP < -1500) {
			OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500.0);
			SENS2 += 4 * (TEMP + 1500) * (TEMP + 1500.0);
		}
	} else {
		T2 = 7 * ((uint64_t) dT * dT) / 137438953472;
		OFF2 = ((TEMP - 2000) * (TEMP - 2000)) / 16;
		SENS2 = 0;
	}
	OFF = ((int64_t) c2 << 16) + (((c4 * (int64_t) dT)) >> 7);
	SENS = ((int64_t) c1 << 15) + (((c3 * (int64_t) dT)) >> 8);

	TEMP -= T2;
	OFF -= OFF2;
	SENS -= SENS2;
	P = ((D1 * SENS) / 2097152 - OFF) / 32768;

	*t = (float) TEMP / 100;
	*p = (float) P / 10;

}

