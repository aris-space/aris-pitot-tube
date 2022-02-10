/*
 * task1.h
 *
 *  Created on: 25 Aug 2020
 *  Author: ARIS / Linus Stoeckli
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

#define STAT_INIT() \
  { \
    .LED_port = STAT_GPIO_Port, \
    .LED_pin = STAT_Pin, \
  }
#define RDY_INIT() \
  { \
    .LED_port = RDY_GPIO_Port, \
    .LED_pin = RDY_Pin, \
  }

// *** structs *** //

typedef struct LED_dev {

	// Hardware Configuration
	GPIO_TypeDef *LED_port;
	uint16_t LED_pin;
} LED;

extern void toggle(struct LED_dev *ctrl);
extern void turn_on(struct LED_dev *ctrl);
extern void turn_off(struct LED_dev *ctrl);

#endif /* INC_LED_H_ */
