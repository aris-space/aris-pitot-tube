/*
 * utils.c
 *
 *  Created on: Sep 20, 2020
 *      Author: linus
 */


#include "utils.h"
#include <math.h>

float idle_detect_buffer[IDLE_DETECT_LEN] = {0.0};


uint8_t device_is_idle(float * a){

	// TODO: include gyro

	for (int i = 1; i < IDLE_DETECT_LEN; i++){
		idle_detect_buffer[i-1] = idle_detect_buffer[i];
	}
	idle_detect_buffer[IDLE_DETECT_LEN-1] = sqrtf(a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);
	float sum_a = 0.0;
	for (int i = 0; i < IDLE_DETECT_LEN; i++){
		sum_a += idle_detect_buffer[i];
	}
	sum_a /= (float)IDLE_DETECT_LEN;

	// if average of acceleration over 5 measurements is around 1G, idle has been detected.
	if ((sum_a <= 1.1) && (sum_a >= 0.9)) return 1;
	return 0;
}

