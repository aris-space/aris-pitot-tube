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

#define IDLE_DETECT_LEN 1000 / CHECK_IDLE_INTERVAL * 10   // 10 seconds
#define LAUNCH_DETECT_LEN 1000 / CHECK_IDLE_INTERVAL * 3  // 3 seconds
#define LAUNCH_THRESHOLD 4  // 4G

#define DATA_CONTAINER_INIT() \
  { \
    .tick = 0, \
    .flight_phase = PAD_IDLE, \
    .accel_x = 0, \
    .accel_y = 0, \
    .accel_z = 0, \
	.gyro_x = 0, \
    .gyro_y = 0, \
    .gyro_z = 0, \
    .accel_t = 2500, \
	.accel_ok = HAL_TIMEOUT, \
	.baro1_D1 = 0, \
	.baro1_D2 = 0, \
	.baro1_ok = HAL_TIMEOUT, \
    .baro2_D1 = 0, \
    .baro2_D2 = 0, \
	.baro2_ok = HAL_TIMEOUT, \
    .temp_th = 2500, \
    .temp_tc = 2500, \
	.temp_ok = HAL_TIMEOUT, \
  }

#define CAL_CONATINER_INIT() \
  { \
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

enum fph {
  PAD_IDLE = 1,
  FLIGHT = 2,
};


typedef struct cal_container {
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
} cal_t;

typedef struct data_container {
	uint32_t tick;
	uint8_t flight_phase;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t accel_t;
	uint8_t accel_ok;
	uint32_t baro1_D1;
	uint32_t baro1_D2;
	uint8_t baro1_ok;
	uint32_t baro2_D1;
	uint32_t baro2_D2;
	uint8_t baro2_ok;
	int16_t temp_th;
	int16_t temp_tc;
	uint8_t temp_ok;
} data_t;

uint8_t device_is_idle(cal_t *LOG, data_t *DATA, uint32_t len);
uint8_t launch_detect(cal_t *LOG, data_t *DATA, uint32_t len);

#endif /* INC_UTILS_H_ */
