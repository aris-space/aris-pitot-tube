/*
 * task1.c
 *
 *  Created on: 25 Aug 2020
 *      Author: linus
 */

#include "devices/LED.h"

void toggle(struct LED_dev * ctrl){
	HAL_GPIO_TogglePin(ctrl->LED_port, ctrl->LED_pin);
}

void turn_on(struct LED_dev * ctrl){
	HAL_GPIO_WritePin(ctrl->LED_port, ctrl->LED_pin, GPIO_PIN_SET);
}

void turn_off(struct LED_dev * ctrl){
	HAL_GPIO_WritePin(ctrl->LED_port, ctrl->LED_pin, GPIO_PIN_RESET);
}
