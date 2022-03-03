/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 * adapted by		   : ARIS / Linus Stoeckli
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "devices/LED.h"
#include "devices/ICM20601.h"
#include "devices/MS5803.h"
#include "devices/MCP9600.h"

#include "SD.h"
#include "utils.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
ICM20601 IMU = IMU_INIT();

MS5803 BARO1 = BARO1_INIT();
MS5803 BARO2 = BARO2_INIT();

MCP9600 TEMP = MCP_INIT();
LED STAT = STAT_INIT();
LED RDY = RDY_INIT();

data_t DATA = DATA_CONTAINER_INIT();
cal_t CAL = CAL_CONATINER_INIT();
accel_data_t accel_data;
baro_data_t baro1_data;
baro_data_t baro2_data;
temp_data_t temp_data;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t good_night_mode = 0;
uint8_t wake_up_mode = 0;

float a[8];
float p1 = 0;
float p2 = 0;
float t1 = 0;
float t2 = 0;
float T[4];

char buffer[BUFLEN]; // to store data
uint16_t num_dat_file = 0;
uint16_t num_dir = 0;
uint8_t SD_state = 0;
uint16_t counter = 0;

char FILE_NAME[24];
char CAL_NAME[24];
char LOG_NAME[24];

char log_buffer[BUFLEN];

volatile uint32_t tick = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for accelTask */
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
  .name = "accelTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for baroTask */
osThreadId_t baroTaskHandle;
const osThreadAttr_t baroTask_attributes = {
  .name = "baroTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for tempTask */
osThreadId_t tempTaskHandle;
const osThreadAttr_t tempTask_attributes = {
  .name = "tempTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SDTask */
osThreadId_t SDTaskHandle;
const osThreadAttr_t SDTask_attributes = {
  .name = "SDTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gatherTask */
osThreadId_t gatherTaskHandle;
const osThreadAttr_t gatherTask_attributes = {
  .name = "gatherTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SDQueue */
osMessageQueueId_t SDQueueHandle;
const osMessageQueueAttr_t SDQueue_attributes = {
  .name = "SDQueue"
};
/* Definitions for baro1_mutex */
osMutexId_t baro1_mutexHandle;
const osMutexAttr_t baro1_mutex_attributes = {
  .name = "baro1_mutex"
};
/* Definitions for baro2_mutex */
osMutexId_t baro2_mutexHandle;
const osMutexAttr_t baro2_mutex_attributes = {
  .name = "baro2_mutex"
};
/* Definitions for accel_mutex */
osMutexId_t accel_mutexHandle;
const osMutexAttr_t accel_mutex_attributes = {
  .name = "accel_mutex"
};
/* Definitions for temp_mutex */
osMutexId_t temp_mutexHandle;
const osMutexAttr_t temp_mutex_attributes = {
  .name = "temp_mutex"
};
/* Definitions for log_mutex */
osMutexId_t log_mutexHandle;
const osMutexAttr_t log_mutex_attributes = {
  .name = "log_mutex"
};
/* Definitions for sleepSem */
osSemaphoreId_t sleepSemHandle;
const osSemaphoreAttr_t sleepSem_attributes = {
  .name = "sleepSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAccelTask(void *argument);
void StartBaroTask(void *argument);
void StartTempTask(void *argument);
void StartSDTask(void *argument);
void StartGatherTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	turn_on(&RDY);
	turn_on(&STAT);

	uint8_t baro1_rdy = ms5803_init(&BARO1);
	while (baro1_rdy != 1) {
		baro1_rdy = ms5803_init(&BARO1);
		osDelay(100);
	}
	CAL.baro1_cal_1 = BARO1.cal[0];
	CAL.baro1_cal_2 = BARO1.cal[1];
	CAL.baro1_cal_3 = BARO1.cal[2];
	CAL.baro1_cal_4 = BARO1.cal[3];
	CAL.baro1_cal_5 = BARO1.cal[4];
	CAL.baro1_cal_6 = BARO1.cal[5];

	uint8_t baro2_rdy = ms5803_init(&BARO2);
	while (baro2_rdy != 1) {
		baro2_rdy = ms5803_init(&BARO2);
		osDelay(100);
	}
	CAL.baro2_cal_1 = BARO2.cal[0];
	CAL.baro2_cal_2 = BARO2.cal[1];
	CAL.baro2_cal_3 = BARO2.cal[2];
	CAL.baro2_cal_4 = BARO2.cal[3];
	CAL.baro2_cal_5 = BARO2.cal[4];
	CAL.baro2_cal_6 = BARO2.cal[5];

	uint8_t imu_rdy = icm20601_init(&IMU);
	while (imu_rdy != 1) {
		imu_rdy = icm20601_init(&IMU);
		osDelay(100);
	}
	CAL.accel_sens = (uint16_t) _get_accel_sensitivity(IMU.accel_g);
	CAL.gyro_sens = (uint16_t) (_get_gyro_sensitivity(IMU.gyro_dps) * 10);

	uint8_t temp_rdy = mcp9600_init(&TEMP);
	while (temp_rdy != 1) {
		temp_rdy = mcp9600_init(&TEMP);
		osDelay(100);
	}

	turn_off(&RDY);
	turn_off(&STAT);

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of baro1_mutex */
  baro1_mutexHandle = osMutexNew(&baro1_mutex_attributes);

  /* creation of baro2_mutex */
  baro2_mutexHandle = osMutexNew(&baro2_mutex_attributes);

  /* creation of accel_mutex */
  accel_mutexHandle = osMutexNew(&accel_mutex_attributes);

  /* creation of temp_mutex */
  temp_mutexHandle = osMutexNew(&temp_mutex_attributes);

  /* creation of log_mutex */
  log_mutexHandle = osMutexNew(&log_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sleepSem */
  sleepSemHandle = osSemaphoreNew(4, 4, &sleepSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SDQueue */
  SDQueueHandle = osMessageQueueNew (100, sizeof(data_t), &SDQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of accelTask */
  accelTaskHandle = osThreadNew(StartAccelTask, NULL, &accelTask_attributes);

  /* creation of baroTask */
  baroTaskHandle = osThreadNew(StartBaroTask, NULL, &baroTask_attributes);

  /* creation of tempTask */
  tempTaskHandle = osThreadNew(StartTempTask, NULL, &tempTask_attributes);

  /* creation of SDTask */
  SDTaskHandle = osThreadNew(StartSDTask, NULL, &SDTask_attributes);

  /* creation of gatherTask */
  gatherTaskHandle = osThreadNew(StartGatherTask, NULL, &gatherTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		// if average of last measurement buffer is < 1.5g
		// and gyros not moving then go into good_night_mode
		if (good_night_mode == 0) {
			if (osMutexAcquire(accel_mutexHandle, ACCEL_MUTEX_TIMEOUT)){
				if (device_is_idle(&CAL, &DATA, IDLE_DETECT_LEN) == 1) {
					if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
						sprintf(log_buffer, "%lu, going to sleep \n", HAL_GetTick());
						log_msg(LOG_NAME,log_buffer);
						osMutexRelease(log_mutexHandle);
					}
					good_night_mode = 1;
				}
				DATA.flight_phase = launch_detect(&CAL, &DATA, LAUNCH_DETECT_LEN);
				osMutexRelease(accel_mutexHandle);
			}
		} else {
			osSemaphoreAcquire(sleepSemHandle, portMAX_DELAY);
			osThreadSuspend(defaultTaskHandle);
		}

		osDelay(CHECK_IDLE_INTERVAL);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAccelTask */
/**
 * @brief Function implementing the accelTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAccelTask */
void StartAccelTask(void *argument)
{
  /* USER CODE BEGIN StartAccelTask */
	float accel_temperature = 0;
	int16_t accel_raw[3] = { 0 };
	int16_t gyro_raw[3] = { 0 };
	uint8_t accel_stat = HAL_TIMEOUT;
	uint8_t sem_count = 0;
	uint32_t tick_count = 0;
	/* Infinite loop */
	for (;;) {

		tick_count = osKernelGetTickCount();
		if (good_night_mode == 1) {
			icm20601_standby(&IMU);
			sem_count = osSemaphoreGetCount(sleepSemHandle);
			while (sem_count != 0) {
				turn_on(&RDY);
				turn_on(&STAT);
				osDelay(50);
				turn_off(&RDY);
				turn_off(&STAT);
				osDelay(50);
				sem_count = osSemaphoreGetCount(sleepSemHandle);
			}
			osDelay(100);
			turn_off(&RDY);
			turn_off(&STAT);
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

			// Clear the WU FLAG
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			HAL_PWR_EnterSTANDBYMode();

			osThreadSuspend(accelTaskHandle);

			// now in stand-by
			// resets up on wake-up

		} else {
			if (DEBUG_PRINT == 1)
				icm20601_read_data(&IMU, a);
			accel_stat = icm20601_read_accel_raw(&IMU, accel_raw);
			accel_stat += icm20601_read_gyro_raw(&IMU, gyro_raw);
			accel_stat += icm20601_read_temp(&IMU, &accel_temperature);
			if (accel_stat == HAL_OK) {
				/* If the Mutex is acquired we write the data into the right variable */
				if (osMutexAcquire(accel_mutexHandle, ACCEL_MUTEX_TIMEOUT)
						== osOK) {
					accel_data.accel_t = (int16_t) (accel_temperature * 100);
					accel_data.accel_x = accel_raw[0];
					accel_data.accel_y = accel_raw[1];
					accel_data.accel_z = accel_raw[2];
					accel_data.gyro_x = gyro_raw[0];
					accel_data.gyro_y = gyro_raw[1];
					accel_data.gyro_z = gyro_raw[2];
					osMutexRelease(accel_mutexHandle);
				}
			} else {
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, error on imu: %d, re-initing\n", HAL_GetTick(), accel_stat);
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);

				}
				uint8_t imu_rdy = icm20601_init(&IMU);
				while (imu_rdy != 1) {
					imu_rdy = icm20601_init(&IMU);
					osDelay(100);
				}
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, imu init success\n", HAL_GetTick());
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);
				}
			}

		}
		osDelayUntil(tick_count + 1);
		// osDelay(IMU_INTERVAL);
	}
  /* USER CODE END StartAccelTask */
}

/* USER CODE BEGIN Header_StartBaroTask */
/**
 * @brief Function implementing the baroTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBaroTask */
void StartBaroTask(void *argument)
{
  /* USER CODE BEGIN StartBaroTask */
	uint8_t baro1_stat = HAL_TIMEOUT;
	uint8_t baro2_stat = HAL_TIMEOUT;

	// wait time is the same for both baros, so just take the first
	uint32_t wait_time = ms5803_get_conversion_ticks(&BARO1);
	/* Infinite loop */
	for (;;) {

		if (good_night_mode == 0) {

			osDelay(wait_time);
			ms5803_prep_pressure(&BARO1);
			ms5803_prep_pressure(&BARO2);

			osDelay(wait_time);
			baro1_stat = ms5803_read_pressure(&BARO1);
			baro2_stat = ms5803_read_pressure(&BARO2);

			if (baro1_stat == HAL_OK) {
				if (osMutexAcquire(baro1_mutexHandle, BARO_MUTEX_TIMEOUT)
						== osOK) {
					baro1_data.baro_D1 = BARO1.D1;
					baro1_data.baro_D2 = BARO1.D2;
					osMutexRelease(baro1_mutexHandle);
				}
			} else {
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, error on baro1: %d, re-initing\n", HAL_GetTick(), baro1_stat);
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);

				}

				uint8_t baro1_rdy = ms5803_init(&BARO1);
				while (baro1_rdy != 1) {
					baro1_rdy = ms5803_init(&BARO1);
					osDelay(100);
				}
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, baro1 init success\n", HAL_GetTick());
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);
				}
			}

			if (baro2_stat == HAL_OK) {
				if (osMutexAcquire(baro2_mutexHandle, BARO_MUTEX_TIMEOUT)
						== osOK) {
					baro2_data.baro_D1 = BARO2.D1;
					baro2_data.baro_D2 = BARO2.D2;
					osMutexRelease(baro2_mutexHandle);
				}
			} else {
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, error on baro2: %d, re-initing\n", HAL_GetTick(), baro2_stat);
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);
				}

				uint8_t baro2_rdy = ms5803_init(&BARO2);
				while (baro2_rdy != 1) {
					baro2_rdy = ms5803_init(&BARO2);
					osDelay(100);
				}
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, baro2 init success\n", HAL_GetTick());
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);
				}
			}

			if (DEBUG_PRINT == 1)
				ms5803_convert(&BARO1, &p1, &t1);
			if (DEBUG_PRINT == 1)
				ms5803_convert(&BARO2, &p2, &t2);
		} else {
			osSemaphoreAcquire(sleepSemHandle, portMAX_DELAY);
			osThreadSuspend(baroTaskHandle);
		}

		osDelay(BARO_INTERVAL);
		toggle(&STAT);
	}
  /* USER CODE END StartBaroTask */
}

/* USER CODE BEGIN Header_StartTempTask */
/**
 * @brief Function implementing the tempTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTempTask */
void StartTempTask(void *argument)
{
  /* USER CODE BEGIN StartTempTask */
	uint8_t temp_stat = HAL_TIMEOUT;

	/* Infinite loop */
	for (;;) {
		if (good_night_mode == 0) {
			temp_stat = mcp9600_read(&TEMP, T);
			if (temp_stat == HAL_OK) {
				if (osMutexAcquire(temp_mutexHandle, TEMP_MUTEX_TIMEOUT)
						== osOK) {
					temp_data.temp_tc = (int16_t) (T[2] * 100);
					temp_data.temp_th = (int16_t) (T[0] * 100);
					osMutexRelease(temp_mutexHandle);
				}
			} else {
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, error on temp: %d, re-initing\n", HAL_GetTick(), temp_stat);
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);

				}

				uint8_t temp_rdy = mcp9600_init(&TEMP);
				while (temp_rdy != 1) {
					temp_rdy = mcp9600_init(&TEMP);
					osDelay(100);
				}
				if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
					sprintf(log_buffer, "%lu, baro1 init success\n", HAL_GetTick());
					log_msg(LOG_NAME,log_buffer);
					osMutexRelease(log_mutexHandle);
				}
			}
		} else {
			osSemaphoreAcquire(sleepSemHandle, portMAX_DELAY);
			osThreadSuspend(tempTaskHandle);
		}

		osDelay(TEMP_INTERVAL);
	}
  /* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartSDTask */
/**
 * @brief Function implementing the SDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSDTask */
void StartSDTask(void *argument)
{
  /* USER CODE BEGIN StartSDTask */
	uint16_t buffer_size = 0;
	uint16_t line_counter = 0;
	FRESULT res = FR_OK;
	uint32_t buffer_len = 0;
	data_t DATA_RECEIVE = DATA_CONTAINER_INIT();

	if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {

		printf("mounting SD card \n");
		mount_sd_card();

		setup_dir(&num_dir);

		num_dat_file = 0;

		sprintf(FILE_NAME, "DAT%04u/FD%04u.BIN", (unsigned int) num_dir,
				(unsigned int) num_dat_file);
		if (DEBUG_PRINT == 1)
			printf("saving %s ... \n", FILE_NAME);

		sprintf(CAL_NAME, "DAT%04u/CAL.BIN", (unsigned int) num_dir);
		if (DEBUG_PRINT == 1)
			printf("saving %s ... \n", CAL_NAME);

		sprintf(LOG_NAME, "DAT%04u/LOG.CSV", (unsigned int) num_dir);
		if (DEBUG_PRINT == 1)
			printf("saving %s ... \n", LOG_NAME);

		init_log(LOG_NAME);

		if (write_cal_file(CAL_NAME, &CAL, &buffer_size) != FR_OK) {
			while (1) {
				toggle(&RDY);
				osDelay(250);
				toggle(&STAT);
				osDelay(250);
			}
		}
		osMutexRelease(log_mutexHandle);
	}
	res = open_file(FILE_NAME);

	/* Infinite loop */
	for (;;) {

		if (good_night_mode == 0) {
			buffer_len = osMessageQueueGetCount(SDQueueHandle);
			printf("queue length: %u \n", (unsigned int)osMessageQueueGetCount(SDQueueHandle));
			printf("SD_BUFFER_LEN: %d \n", (int)SD_BUFFER_LEN);

			if (buffer_len  < (int)SD_BUFFER_LEN - 1){
				osDelay(1);
				printf("queue too short \n");
				continue;
			} else {
				for (int j = 0; j < (int)SD_BUFFER_LEN; j++){
					printf("saving queue entry %d \n", j);

					osMessageQueueGet(SDQueueHandle, &DATA_RECEIVE, 0U, 0U);

					res = write_to_file(&DATA_RECEIVE, &buffer_size);
					if (buffer_size > WRITE_BUFFER_SIZE) {
						// res = flush_buffer();
						buffer_size = 0;
					}

					line_counter++;
					if (line_counter > SECONDS_PER_FILE * 1000 / 1) {
						turn_on(&RDY);
						buffer_size = 0;
						line_counter = 0;
						num_dat_file++;
						res = flush_buffer();
						close_file();
						sprintf(FILE_NAME, "DAT%04u/FD%04u.BIN", (unsigned int) num_dir,
								(unsigned int) num_dat_file);
						res = open_file(FILE_NAME);
						turn_off(&RDY);
					}
					if (DEBUG_PRINT == 1)
						printf("tick: %ld \n", tick);
					if (DEBUG_PRINT == 1)
						printf("flight phase: %u \n", DATA_RECEIVE.flight_phase);
					if (DEBUG_PRINT == 1)
						printf("T_a: %4.2f C \n", a[0]);
					if (DEBUG_PRINT == 1)
						printf("ax: %4.2f m/s2 \n", a[1]);
					if (DEBUG_PRINT == 1)
						printf("ay: %4.2f m/s2 \n", a[2]);
					if (DEBUG_PRINT == 1)
						printf("az: %4.2f m/s2 \n", a[3]);
					if (DEBUG_PRINT == 1)
						printf("gx: %4.2f dps \n", a[4]);
					if (DEBUG_PRINT == 1)
						printf("gy: %4.2f dps \n", a[5]);
					if (DEBUG_PRINT == 1)
						printf("gz: %4.2f dps \n", a[6]);
					if (DEBUG_PRINT == 1)
						printf("p1: %4.2f mBar \n", p1);
					if (DEBUG_PRINT == 1)
						printf("p2: %4.2f mBar \n", p2);
					if (DEBUG_PRINT == 1)
						printf("p_t1: %4.2f C \n", t1);
					if (DEBUG_PRINT == 1)
						printf("p_t2: %4.2f C \n", t2);
					if (DEBUG_PRINT == 1)
						printf("T_H: %4.2f C \n", T[0]);
					if (DEBUG_PRINT == 1)
						printf("T_D: %4.2f C \n", T[1]);
					if (DEBUG_PRINT == 1)
						printf("T_C: %4.2f C \n", T[2]);
				}
			}
		} else {
			close_file();
			unmount_sd_card();
			osSemaphoreAcquire(sleepSemHandle, portMAX_DELAY);
			osThreadSuspend(SDTaskHandle);
		}
		if (res != FR_OK){
			remount_sd_card();
			if (osMutexAcquire(log_mutexHandle, LOG_MUTEX_TIMEOUT) == osOK) {
				sprintf(log_buffer, "%lu, error on SD card: %d, re-mounted...\n", HAL_GetTick(), res);
				log_msg(LOG_NAME,log_buffer);
				osMutexRelease(log_mutexHandle);
			}
		}
		osDelay(SAVE_INTERVAL);
	}
  /* USER CODE END StartSDTask */
}

/* USER CODE BEGIN Header_StartGatherTask */
/**
* @brief Function implementing the gatherTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGatherTask */
void StartGatherTask(void *argument)
{
  /* USER CODE BEGIN StartGatherTask */
  /* Infinite loop */
  uint32_t tick_count = 0;
  osStatus_t ret = osOK;
  for(;;)
  {
	tick_count = osKernelGetTickCount();
	DATA.tick = tick_count;

	if (osMutexAcquire(accel_mutexHandle, ACCEL_MUTEX_TIMEOUT)
			== osOK) {
		DATA.accel_t = accel_data.accel_t;
		DATA.accel_x = accel_data.accel_x;
		DATA.accel_y = accel_data.accel_y;
		DATA.accel_z = accel_data.accel_z;
		DATA.gyro_x = accel_data.gyro_x;
		DATA.gyro_y = accel_data.gyro_y;
		DATA.gyro_z = accel_data.gyro_z;
		osMutexRelease(accel_mutexHandle);
	}

	if (osMutexAcquire(baro1_mutexHandle, BARO_MUTEX_TIMEOUT) == osOK) {
		DATA.baro1_D1 = baro1_data.baro_D1;
		DATA.baro1_D2 = baro1_data.baro_D2;
		osMutexRelease(baro1_mutexHandle);
	}

	if (osMutexAcquire(baro2_mutexHandle, BARO_MUTEX_TIMEOUT) == osOK) {
		DATA.baro2_D1 = baro2_data.baro_D1;
		DATA.baro2_D2 = baro2_data.baro_D2;
		osMutexRelease(baro2_mutexHandle);
	}

	if (osMutexAcquire(temp_mutexHandle, TEMP_MUTEX_TIMEOUT) == osOK) {
		DATA.temp_tc = temp_data.temp_tc;
		DATA.temp_th = temp_data.temp_th;
		osMutexRelease(temp_mutexHandle);
	}

	ret = osMessageQueuePut(SDQueueHandle, &DATA, 0U, 0U);
	if (DEBUG_PRINT == 1)
		printf("queue size: %u, status %d\n", (unsigned int)osMessageQueueGetCount(SDQueueHandle), ret);

    osDelayUntil(tick_count + 1);
  }
  /* USER CODE END StartGatherTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

