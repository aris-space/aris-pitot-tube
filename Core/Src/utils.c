/*
 * utils.c
 *
 *  Created on: Sep 20, 2020
 *      Author: linus
 */

#include "utils.h"
#include <math.h>

uint16_t idle_detect_buffer_a[IDLE_DETECT_LEN] = { 0.0 };
uint16_t idle_detect_buffer_g[IDLE_DETECT_LEN] = { 0.0 };
uint16_t launch_detect_buffer[LAUNCH_DETECT_LEN] = { 0.0 };
uint32_t empty_idle_buffer_counter = 0;
uint32_t empty_launch_buffer_counter = 0;

uint8_t device_is_idle(cal_t *LOG, data_t *DATA, uint32_t len) {
	float a[3] = { 0.0 };
	float g[3] = { 0.0 };

	a[0] = ((float) DATA->accel_x) / LOG->accel_sens;
	a[1] = ((float) DATA->accel_y) / LOG->accel_sens;
	a[2] = ((float) DATA->accel_z) / LOG->accel_sens;

	g[0] = ((float) DATA->gyro_x) / LOG->gyro_sens;
	g[1] = ((float) DATA->gyro_y) / LOG->gyro_sens;
	g[2] = ((float) DATA->gyro_z) / LOG->gyro_sens;

	for (int i = 1; i < IDLE_DETECT_LEN; i++) {
		idle_detect_buffer_a[i - 1] = idle_detect_buffer_a[i];
		idle_detect_buffer_g[i - 1] = idle_detect_buffer_g[i];
	}
	idle_detect_buffer_a[IDLE_DETECT_LEN - 1] = (uint16_t) (1000
			* sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
	idle_detect_buffer_g[IDLE_DETECT_LEN - 1] = (uint16_t) (1000
			* sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]));
	float sum_a = 0.0;
	float sum_g = 0.0;
	for (int i = IDLE_DETECT_LEN - len; i < IDLE_DETECT_LEN; i++) {
		sum_a += idle_detect_buffer_a[i];
	}
	for (int i = IDLE_DETECT_LEN - len; i < IDLE_DETECT_LEN; i++) {
		sum_g += idle_detect_buffer_g[i];
	}
	sum_a /= (float) len;
	sum_a /= 1000;
	sum_g /= (float) len;
	sum_g /= 1000;
	printf("sum_g: %4.2f \n", sum_g);
	if (empty_idle_buffer_counter <= IDLE_DETECT_LEN)
		empty_idle_buffer_counter++;
	if (empty_idle_buffer_counter < IDLE_DETECT_LEN) {
		printf("still filling buffer... \n");
		return 0;
	}
	if ((0.5 <= sum_a) && (sum_a <= 1.5) && (sum_g <= 0.5))
		return 1;
	return 0;
}

uint8_t launch_detect(cal_t *LOG, data_t *DATA, uint32_t len) {
	float a[3] = { 0.0 };

	a[0] = ((float) DATA->accel_x) / LOG->accel_sens;
	a[1] = ((float) DATA->accel_y) / LOG->accel_sens;
	a[2] = ((float) DATA->accel_z) / LOG->accel_sens;

	for (int i = 1; i < LAUNCH_DETECT_LEN; i++) {
		launch_detect_buffer[i - 1] = launch_detect_buffer[i];
	}
	launch_detect_buffer[IDLE_DETECT_LEN - 1] = (uint16_t) (1000
			* sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
	float sum_a = 0.0;
	for (int i = LAUNCH_DETECT_LEN - len; i < LAUNCH_DETECT_LEN; i++) {
		sum_a += launch_detect_buffer[i];
	}
	sum_a /= (float) len;
	sum_a /= 1000;
	if (empty_launch_buffer_counter <= LAUNCH_DETECT_LEN)
		empty_launch_buffer_counter++;
	if (empty_launch_buffer_counter < LAUNCH_DETECT_LEN)
		return PAD_IDLE;
	if (sum_a >= LAUNCH_THRESHOLD)
		return FLIGHT;
	return PAD_IDLE;
}

