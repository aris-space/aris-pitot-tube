/*
 * MCP9600.h
 *
 *  Created on: 17 Jun 2020
 *  Author: ARIS / Linus Stoeckli
 */

#ifndef MCP9600_H_
#define MCP9600_H_

#include "main.h"

#define MCP_SAMPLING 1000

#define MCP_INIT() \
  { \
    .addr = 0x60 << 1, \
	.i2c_bus = &hi2c2, \
	.active = 1, \
	.delay = 100, \
	.last_call = 0, \
  }

typedef struct mcp9600_dev {
	// Hardware Configuration
	uint8_t addr;
	I2C_HandleTypeDef* i2c_bus;
	uint8_t active;
	uint8_t delay;
	uint32_t last_call;
} MCP9600;

int mcp9600_init(struct mcp9600_dev * dev);
void mcp9600_read(struct mcp9600_dev * dev, float * dat);

void write(struct mcp9600_dev * dev, uint8_t REG, uint8_t val);

#endif /* MCP9600_H_ */
