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

#define BARO1_INIT() \
  { \
    .addr = 0x76 << 1, \
	.i2c_bus = &hi2c1, \
	.active = 1, \
	.delay = 100, \
  }

#define BARO2_INIT() \
  { \
    .addr = 0x77 << 1, \
	.i2c_bus = &hi2c1, \
	.active = 1, \
	.delay = 100, \
  }

enum ms5803_stage {
  MS_TEMPERATURE_REQ = 0,
  MS_PRESSURE_REQ = 1,
};


// *** structs *** //

typedef struct ms5803_dev {

	// Hardware Configuration
	uint8_t addr;
	I2C_HandleTypeDef* i2c_bus;
	uint8_t active;
	uint8_t delay;
	uint16_t cal[6];
	uint32_t D1;
	uint32_t D2;
} MS5803;

extern uint8_t ms5803_init(struct ms5803_dev * dev);
extern void ms5803_prep_pressure(struct ms5803_dev * dev);
extern void ms5803_read_pressure(struct ms5803_dev * dev);
extern void ms5803_convert(struct ms5803_dev * dev, float * p, float * t);


#endif /* MS5803_H_ */
