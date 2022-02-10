// ICM-20601 IMU Device Library
// Author: Luca Jost
// Adapted by Linus Stoeckli (5.2.2022)
// 11.06.2020

// *** Includes *** //

#include "devices/ICM20601.h"
#include "cmsis_os.h"

// *** Macros *** //

#define UINT8_TO_INT16(dst, src_high, src_low) \
  do { \
    dst = (src_high); \
    dst <<= 8; \
    dst |= (src_low); \
  } while (0);

static float temperature_sensitivity = 326.8;

// *** Local functions *** //

// Used to convert raw accelerometer readings to G-force.
float _get_accel_sensitivity(enum icm20601_accel_g accel_g) {
	float f = 0.0;

	switch (accel_g) {
	case (ICM20601_ACCEL_RANGE_4G):
		f = 8192.0;
		break;
	case (ICM20601_ACCEL_RANGE_8G):
		f = 4096.0;
		break;
	case (ICM20601_ACCEL_RANGE_16G):
		f = 2048.0;
		break;
	case (ICM20601_ACCEL_RANGE_32G):
		f = 1024.0;
		break;
	}
	return f;
}

// Used to convert raw gyroscope readings to degrees per second.
float _get_gyro_sensitivity(enum icm20601_gyro_dps gyro_dps) {
	float f = 0;

	switch (gyro_dps) {
	case (ICM20601_GYRO_RANGE_500_DPS):
		f = 65.5;
		break;
	case (ICM20601_GYRO_RANGE_1000_DPS):
		f = 32.8;
		break;
	case (ICM20601_GYRO_RANGE_2000_DPS):
		f = 16.4;
		break;
	case (ICM20601_GYRO_RANGE_4000_DPS):
		f = 8.2;
		break;
	}
	return f;
}

uint8_t _compareArrays(uint8_t *a, uint8_t *b, int size) {
	for (int i = 1; i <= size; i++) {
		if (a[i] != b[i])
			return 0;
	}
	return 1;
}

// Read bytes from MEMS
void _icm_read_bytes(struct icm20601_dev *dev, uint8_t reg, uint8_t *pData,
		uint16_t size) {
	reg = reg | 0x80;
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->spi_bus, &reg, 1, IMU20601_SPI_TIMEOUT);
	HAL_SPI_Receive(dev->spi_bus, pData, size, IMU20601_SPI_TIMEOUT);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// Write bytes to MEMS
void _icm_write_bytes(struct icm20601_dev *dev, uint8_t reg, uint8_t *pData,
		uint16_t size) {

	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->spi_bus, &reg, 1, IMU20601_SPI_TIMEOUT);
	HAL_SPI_Transmit(dev->spi_bus, pData, size, IMU20601_SPI_TIMEOUT);
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

// *** Global Functions *** //

int8_t icm20601_init(struct icm20601_dev *dev) {
	uint8_t tmp = 0;
	uint8_t r[1] = { 0 };

	// General Procedure:
	//  1. reset chip
	//  2. set clock for PLL for optimum performance as documented in datasheet
	//  3. place accelerometer and gyroscope into standby
	//  4. disable fifo
	//  5. configure chip
	//  6. enable accelerometer and gyroscope

	// full reset of chip
	tmp = SENS_reset; // 0x81
	_icm_write_bytes(dev, REG_PWR_MGMT_1, &tmp, 1);
	HAL_Delay(1);

	// set clock to internal PLL
	tmp = SENS_internalpll; //0x01
	_icm_write_bytes(dev, REG_PWR_MGMT_1, &tmp, 1);
	HAL_Delay(1);

	// verify we are able to read from the chip
	_icm_read_bytes(dev, REG_WHO_AM_I, r, 1);
	if (r[0] != REG_WHO_AM_I_CONST) {
		printf("IMU setup failed!\n");
		return 0;
	}

	// place accel and gyro on standby
	tmp = SENS_standby; // 0x3F
	_icm_write_bytes(dev, REG_PWR_MGMT_2, &tmp, 1);

	// disable fifo
	tmp = SENS_nofifo; //0x00
	_icm_write_bytes(dev, REG_USER_CTRL, &tmp, 1);

	// disable chip I2C communications
	tmp = SENS_disablei2c;	//0x41;
	_icm_write_bytes(dev, REG_USER_CTRL, &tmp, 1);

	// Accelerometer filtering
	if (ICM20601_ACCEL_DLPF_BYPASS_1046_HZ == dev->accel_dlpf) {
		tmp = (0x01 << 3);
	} else {
		tmp = dev->accel_dlpf;
	}
	_icm_write_bytes(dev, REG_ACCEL_CONFIG_2, &tmp, 1);

	// Accelerometer range
	tmp = (dev->accel_g) << 3;
	_icm_write_bytes(dev, REG_ACCEL_CONFIG_1, &tmp, 1);

	// Gyro filtering
	//tmp = ((dev->gyro_dps) << 3) | SENS_gyrofilter; // filter: 0x02
	//_icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp , 1);

	if (ICM20601_GYRO_DLPF_BYPASS_3281_HZ == dev->gyro_dlpf) {
		// bypass dpf and set dps
		tmp = 0x00;
		_icm_write_bytes(dev, REG_CONFIG, &tmp, 1);

		tmp = (dev->gyro_dps << 3) | 0x02;
		_icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp, 1);
	} else if (ICM20601_GYRO_DLPF_BYPASS_8173_HZ == dev->gyro_dlpf) {
		// bypass dpf and set dps
		tmp = 0x00;
		_icm_write_bytes(dev, REG_CONFIG, &tmp, 1);

		tmp = (dev->gyro_dps << 3) | 0x01;
		_icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp, 1);
	} else {
		// configure dpf and set dps
		tmp = dev->gyro_dlpf;
		_icm_write_bytes(dev, REG_CONFIG, &tmp, 1);

		tmp = dev->gyro_dps << 3;
		_icm_write_bytes(dev, REG_GYRO_CONFIG, &tmp, 1);
	}

	tmp = 0x00;
	_icm_write_bytes(dev, REG_PWR_MGMT_2, &tmp, 1);

	printf("IMU setup successful.\n");

	return 1;
}

int8_t icm20601_standby(struct icm20601_dev *dev) {
	uint8_t tmp = 0;
	uint8_t r[1] = { 0 };

	dev->active = 0;

	_icm_read_bytes(dev, REG_PWR_MGMT_1, r, 1);
	printf("read  REG_PWR_MGMT_1: %d \n", r[0]);

	tmp = 0b00001111 & r[0];
	tmp = 0b00000001;
	printf("writing  REG_PWR_MGMT_1: %d \n", tmp);

	_icm_write_bytes(dev, REG_PWR_MGMT_1, &tmp, 1);

	_icm_read_bytes(dev, REG_PWR_MGMT_1, r, 1);
	printf("read  REG_PWR_MGMT_1: %d \n", r[0]);

	tmp = 0b00000111;
	_icm_write_bytes(dev, REG_PWR_MGMT_2, &tmp, 1);
	osDelay(1);

	_icm_read_bytes(dev, REG_ACCEL_CONFIG_2, r, 1);
	osDelay(1);
	printf("read  REG_ACCEL_CONFIG_2: %d \n", r[0]);

	tmp = 0x01 | (r[0] & 0b11110000);
	tmp = 0b00000101;
	printf("writing  REG_ACCEL_CONFIG_2: %d \n", tmp);

	_icm_write_bytes(dev, REG_ACCEL_CONFIG_2, &tmp, 1);
	osDelay(1);

	tmp = 0b11100000;
	_icm_write_bytes(dev, REG_INT_ENABLE, &tmp, 1);
	osDelay(1);

	tmp = 0x00;
	tmp = 0x7f;
	tmp = 0x19;
	tmp = 100;
	_icm_write_bytes(dev, REG_ACCEL_WOM_THR, &tmp, 1);
	osDelay(1);

	_icm_read_bytes(dev, REG_ACCEL_WOM_THR, r, 1);
	osDelay(1);
	printf("read  REG_ACCEL_WOM_THR: %d \n", r[0]);

	tmp = 0b11000000;
	_icm_write_bytes(dev, REG_ACCEL_INTEL_CTRL, &tmp, 1);
	osDelay(1);

	tmp = 0b00001011;
	_icm_write_bytes(dev, REG_LP_MODE_CFG, &tmp, 1);
	osDelay(1);

	_icm_read_bytes(dev, REG_PWR_MGMT_1, r, 1);
	osDelay(1);
	printf("read  REG_PWR_MGMT_1: %d \n", r[0]);

	_icm_read_bytes(dev, REG_SMPLRT_DIV, r, 1);
	osDelay(1);
	printf("read  REG_SMPLRT_DIV: %d \n", r[0]);

	tmp = 0b00100000;
	printf("writing  REG_PWR_MGMT_1: %d \n", tmp);

	_icm_write_bytes(dev, REG_PWR_MGMT_1, &tmp, 1);
	osDelay(1);

	_icm_read_bytes(dev, REG_INT_PIN_CFG, r, 1);
	osDelay(1);
	printf("read  REG_INT_PIN_CFG: %d \n", r[0]);

	tmp = 0b00000000;
	_icm_write_bytes(dev, REG_INT_PIN_CFG, &tmp, 1);
	osDelay(1);
	osDelay(100);

	_icm_read_bytes(dev, 0x68, r, 1);
	osDelay(1);
	printf("read  0x68: %d \n", r[0]);

	return 1;
}

void icm20601_read_int(struct icm20601_dev *dev, uint8_t *r) {
	_icm_read_bytes(dev, 0x3A, r, 1);
	osDelay(1);
}

// Read out raw acceleration data
void icm20601_read_accel_raw(struct icm20601_dev *dev, int16_t *accel) {
	uint8_t accel_8bit[6] = { 0 };
	_icm_read_bytes(dev, REG_ACCEL_XOUT_H, accel_8bit, 6);

	UINT8_TO_INT16(accel[0], accel_8bit[0], accel_8bit[1]);
	UINT8_TO_INT16(accel[1], accel_8bit[2], accel_8bit[3]);
	UINT8_TO_INT16(accel[2], accel_8bit[4], accel_8bit[5]);
}

// Read out processed acceleration data
void icm20601_read_accel(struct icm20601_dev *dev, float *accel) {
	float accel_sensitivity;
	int16_t accel_raw[3] = { 0 };

	accel_sensitivity = _get_accel_sensitivity(dev->accel_g);

	icm20601_read_accel_raw(dev, accel_raw);

	accel[0] = ((float) accel_raw[0]) / accel_sensitivity;
	accel[1] = ((float) accel_raw[1]) / accel_sensitivity;
	accel[2] = ((float) accel_raw[2]) / accel_sensitivity;
}

// Read out raw gyro data
void icm20601_read_gyro_raw(struct icm20601_dev *dev, int16_t *gyro) {
	uint8_t gyro_8bit[6] = { 0 };
	_icm_read_bytes(dev, REG_GYRO_XOUT_H, gyro_8bit, 6);

	UINT8_TO_INT16(gyro[0], gyro_8bit[0], gyro_8bit[1]);
	UINT8_TO_INT16(gyro[1], gyro_8bit[2], gyro_8bit[3]);
	UINT8_TO_INT16(gyro[2], gyro_8bit[4], gyro_8bit[5]);
}

// Read out processed gyro data
void icm20601_read_gyro(struct icm20601_dev *dev, float *gyro) {
	float gyro_sensitivity;
	int16_t gyro_raw[3] = { 0 };

	gyro_sensitivity = _get_gyro_sensitivity(dev->gyro_dps);

	icm20601_read_gyro_raw(dev, gyro_raw);

	gyro[0] = ((float) gyro_raw[0]) / gyro_sensitivity;
	gyro[1] = ((float) gyro_raw[1]) / gyro_sensitivity;
	gyro[2] = ((float) gyro_raw[2]) / gyro_sensitivity;
}

// Read out raw temperature data
void icm20601_read_temp_raw(struct icm20601_dev *dev, int16_t *temp) {
	uint8_t temp_8bit[2] = { 0 };
	_icm_read_bytes(dev, REG_TEMP_OUT_H, temp_8bit, 2);

	UINT8_TO_INT16(*temp, temp_8bit[0], temp_8bit[1]);
}

// Read out processed temperature in degC
void icm20601_read_temp(struct icm20601_dev *dev, float *temp) {
	int16_t temperature_raw;
	icm20601_read_temp_raw(dev, &temperature_raw);

	*temp = ((float) temperature_raw) / temperature_sensitivity + 25.0; // TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 25degC
}

void icm20601_read_data(struct icm20601_dev *dev, float *buf) {
	int16_t temperature_raw;
	icm20601_read_temp_raw(dev, &temperature_raw);
	buf[0] = ((float) temperature_raw) / temperature_sensitivity + 25.0;
	float accel_sensitivity;
	int16_t accel_raw[3] = { 0 };

	accel_sensitivity = _get_accel_sensitivity(dev->accel_g);

	icm20601_read_accel_raw(dev, accel_raw);

	buf[1] = ((float) accel_raw[0]) / accel_sensitivity * 9.81;
	buf[2] = ((float) accel_raw[1]) / accel_sensitivity * 9.81;
	buf[3] = ((float) accel_raw[2]) / accel_sensitivity * 9.81;

	float gyro_sensitivity;
	int16_t gyro_raw[3] = { 0 };

	gyro_sensitivity = _get_gyro_sensitivity(dev->gyro_dps);

	icm20601_read_gyro_raw(dev, gyro_raw);

	buf[4] = ((float) gyro_raw[0]) / gyro_sensitivity;
	buf[5] = ((float) gyro_raw[1]) / gyro_sensitivity;
	buf[6] = ((float) gyro_raw[2]) / gyro_sensitivity;
}

