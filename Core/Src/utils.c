/*
 * utils.c
 *
 *  Created on: Sep 20, 2020
 *      Author: linus
 */


#include "utils.h"
#include <math.h>

float launch_detect_buffer[LD_LEN] = {0.0};

uint8_t launch_detect(float * a){
	for (int i = 1; i < LD_LEN; i++){
		launch_detect_buffer[i-1] = launch_detect_buffer[i];
	}
	launch_detect_buffer[LD_LEN-1] = sqrtf(a[1]*a[1] + a[2]*a[2] + a[3]*a[3]);
	float sum_a = 0.0;
	for (int i = 0; i < LD_LEN; i++){
		sum_a += launch_detect_buffer[i];
	}
	sum_a /= (float)LD_LEN;

	// if average of acceleration over 5 measurements is higher than 4G, launch has been detected.
	if (sum_a >= 20) return 1;
	return 0;
}
