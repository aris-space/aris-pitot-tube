/*
 * utils.h
 *
 *  Created on: Sep 20, 2020
 *      Author: linus
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include "FreeRTOSConfig.h"

#define IDLE_DETECT_LEN 1000 / CHECK_IDLE_INTERVAL * 10

#define CONTAINER_INIT() \
  { \
    .tick = 0, \
    .accel_x = 0, \
    .accel_y = 0, \
    .accel_z = 0, \
    .accel_t = 2500, \
	.baro1_D1 = 0, \
	.baro1_D2 = 0, \
    .baro2_D1 = 0, \
    .baro2_D2 = 0, \
    .temp_th = 2500, \
    .temp_td = 0, \
  }

#define LOG_INIT() \
  { \
    .file_number = 0, \
    .baro1_cal_1 = 0, \
    .baro1_cal_2 = 0, \
    .baro1_cal_3 = 0, \
    .baro1_cal_4 = 0, \
	.baro1_cal_5 = 0, \
	.baro1_cal_6 = 0, \
    .baro2_cal_1 = 0, \
    .baro2_cal_2 = 0, \
    .baro2_cal_3 = 0, \
    .baro2_cal_4 = 0, \
	.baro2_cal_5 = 0, \
	.baro2_cal_6 = 0, \
    .accel_sens = 0, \
    .gyro_sens = 0, \
  }

typedef struct log_container {
	uint16_t file_number;
	uint16_t baro1_cal_1;
	uint16_t baro1_cal_2;
	uint16_t baro1_cal_3;
	uint16_t baro1_cal_4;
	uint16_t baro1_cal_5;
	uint16_t baro1_cal_6;
	uint16_t baro2_cal_1;
	uint16_t baro2_cal_2;
	uint16_t baro2_cal_3;
	uint16_t baro2_cal_4;
	uint16_t baro2_cal_5;
	uint16_t baro2_cal_6;
	uint16_t accel_sens;
	uint16_t gyro_sens;
} log_t;

typedef struct data_container {
	uint32_t tick;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t accel_t;
	uint32_t baro1_D1;
	uint32_t baro1_D2;
	uint32_t baro2_D1;
	uint32_t baro2_D2;
	int16_t temp_th;
	int16_t temp_td;
} data_t;

uint8_t device_is_idle(float * a);

#endif /* INC_UTILS_H_ */
