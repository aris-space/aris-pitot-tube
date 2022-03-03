/*
 * MS5803.h
 *
 *  Created on: 17 Jun 2020
 *  Author: ARIS / Linus Stoeckli
 */

#ifndef MS5803_H_
#define MS5803_H_

#include "main.h"
#include "i2c.h"

// Commands
#define COMMAND_RESET 0x1E
#define COMMAND_CONVERT_D1_BASE 0x40
#define COMMAND_CONVERT_D2_BASE 0x50
#define COMMAND_ADC_READ 0x00
#define COMMAND_PROM_READ_BASE 0xA0

// Conversion time
#define BARO_CONVERSION_TIME_OSR_BASE 0.6

#define BARO1_INIT() \
  { \
    .addr = 0x76 << 1, \
	.i2c_bus = &hi2c1, \
	.active = 1, \
	.delay = 100, \
	.osr = MS5607_OSR_1024, \
  }

#define BARO2_INIT() \
  { \
    .addr = 0x77 << 1, \
	.i2c_bus = &hi2c1, \
	.active = 1, \
	.delay = 100, \
	.osr = MS5607_OSR_1024, \
  }

enum ms5607_osr {
  MS5607_OSR_256 = 0,
  MS5607_OSR_512 = 1,
  MS5607_OSR_1024 = 2,
  MS5607_OSR_2048 = 3,
  MS5607_OSR_4096 = 4,
};

// *** structs *** //

typedef struct ms5803_dev {

	// Hardware Configuration
	uint8_t addr;
	I2C_HandleTypeDef *i2c_bus;
	uint8_t osr;
	uint8_t active;
	uint8_t delay;
	uint16_t cal[6];
	uint32_t D1;
	uint32_t D2;
} MS5803;

extern uint32_t ms5803_get_conversion_ticks(struct ms5803_dev * dev);
extern uint8_t ms5803_init(struct ms5803_dev *dev);
extern uint8_t ms5803_prep_pressure(struct ms5803_dev *dev);
extern uint8_t ms5803_read_pressure(struct ms5803_dev *dev);
extern void ms5803_convert(struct ms5803_dev *dev, float *p, float *t);

#endif /* MS5803_H_ */
