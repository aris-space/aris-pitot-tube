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

data_t DATA = CONTAINER_INIT();
log_t LOG = LOG_INIT();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t good_night_mode = 0;
uint8_t wake_up_mode = 0;

uint8_t baro1_rdy = 0;
uint8_t baro2_rdy = 0;
uint8_t imu_rdy = 0;
uint8_t temp_rdy = 0;

uint8_t flight_phase = PAD_IDLE;
float a[8];
float p1 = 0;
float p2 = 0;
float t1 = 0;
float t2 = 0;
float T[4];

char buffer[BUFLEN]; // to store data
uint16_t num_dat_file = 0;
uint16_t num_log_file = 0;
uint8_t SD_state = 0;
uint16_t counter = 0;

char FILE_NAME[11];
char LOG_NAME[10];

volatile uint32_t tick = 0;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for accelTask */
osThreadId_t accelTaskHandle;
const osThreadAttr_t accelTask_attributes = {
  .name = "accelTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for baroTask */
osThreadId_t baroTaskHandle;
const osThreadAttr_t baroTask_attributes = {
  .name = "baroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for tempTask */
osThreadId_t tempTaskHandle;
const osThreadAttr_t tempTask_attributes = {
  .name = "tempTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for SDTask */
osThreadId_t SDTaskHandle;
const osThreadAttr_t SDTask_attributes = {
  .name = "SDTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAccelTask(void *argument);
void StartBaroTask(void *argument);
void StartTempTask(void *argument);
void StartSDTask(void *argument);

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

	baro1_rdy = ms5803_init(&BARO1);
	while (baro1_rdy != 1){
		baro1_rdy = ms5803_init(&BARO1);
		osDelay(100);
	}
	baro2_rdy = ms5803_init(&BARO2);
	while (baro2_rdy != 1){
		baro2_rdy = ms5803_init(&BARO2);
		osDelay(100);
	}
	imu_rdy = icm20601_init(&IMU);
	while (imu_rdy != 1){
		imu_rdy = icm20601_init(&IMU);
		osDelay(100);
	}
	temp_rdy = mcp9600_init(&TEMP);
	while (temp_rdy != 1){
		temp_rdy = mcp9600_init(&TEMP);
		osDelay(100);
	}

	uint16_t buffer_size = 0;
	get_file_numbers(&num_dat_file, &num_log_file);
	num_log_file ++;
	num_dat_file ++;

	sprintf(FILE_NAME,"FD%04u.BIN", num_dat_file);
	if (DEBUG_PRINT == 1) printf("saving %s ...",FILE_NAME);

	sprintf(LOG_NAME,"LOG%02u.BIN", num_log_file);
	if (DEBUG_PRINT == 1) printf("saving %s ...",LOG_NAME);

	LOG.file_number = num_dat_file;

	LOG.baro1_cal_1 = BARO1.cal[0];
	LOG.baro1_cal_2 = BARO1.cal[1];
	LOG.baro1_cal_3 = BARO1.cal[2];
	LOG.baro1_cal_4 = BARO1.cal[3];
	LOG.baro1_cal_5 = BARO1.cal[4];
	LOG.baro1_cal_6 = BARO1.cal[5];

	LOG.baro2_cal_1 = BARO2.cal[0];
	LOG.baro2_cal_2 = BARO2.cal[1];
	LOG.baro2_cal_3 = BARO2.cal[2];
	LOG.baro2_cal_4 = BARO2.cal[3];
	LOG.baro2_cal_5 = BARO2.cal[4];
	LOG.baro2_cal_6 = BARO2.cal[5];

	LOG.accel_sens = (uint16_t)_get_accel_sensitivity(IMU.accel_g);
	LOG.gyro_sens = (uint16_t)(_get_gyro_sensitivity(IMU.gyro_dps) * 10);

	while (write_log_file(LOG_NAME, &LOG, &buffer_size) == FR_OK){
		toggle(&RDY);
		osDelay(250);
		toggle(&STAT);
		osDelay(250);
	}


	turn_off(&RDY);
	turn_off(&STAT);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

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
  for(;;)
  {
	// if average of last 2048 is < 1.1g and > 0.9g and gyros not moving then go into good_night_mode
	if (device_is_idle(a) == 1)
	{
		good_night_mode = 1;
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
  /* Infinite loop */
  for(;;)
  {

    if (good_night_mode == 1){
    	icm20601_standby(&IMU);
    	// give other tasks time to stop
    	for (int i=0; i < 10; i ++){
    		  turn_on(&RDY);
    		  turn_on(&STAT);
    		  osDelay(50);
    		  turn_off(&RDY);
			  turn_off(&STAT);
			  osDelay(50);
    	}
    	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
		/* Clear the WU FLAG */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    	HAL_PWR_EnterSTANDBYMode();

    	// now in stand-by
    	// resets up on wake-up

    } else {
    	icm20601_read_data(&IMU, a);
    	DATA.accel_t = (int16_t)(a[0] * 100);
    	DATA.accel_x = (int16_t)(a[1] * 100);
    	DATA.accel_y = (int16_t)(a[2] * 100);
    	DATA.accel_z = (int16_t)(a[3] * 100);
    }

    osDelay(IMU_INTERVAL);

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

  /* Infinite loop */
  for(;;)
  {

	  if (good_night_mode == 0){

			osDelay(3);
			ms5803_prep_pressure(&BARO1);
			ms5803_prep_pressure(&BARO2);

			osDelay(3);
			ms5803_read_pressure(&BARO1);
			ms5803_read_pressure(&BARO2);

			DATA.baro1_D1 = BARO1.D1;
			DATA.baro1_D2 = BARO1.D2;

			DATA.baro2_D1 = BARO2.D1;
			DATA.baro2_D2 = BARO2.D2;

			ms5803_convert(&BARO1, &p1, &t1);
			ms5803_convert(&BARO2, &p2, &t2);
  	  }

   	  osDelay(BARO_INTERVAL);
   	  toggle(&RDY);
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
  /* Infinite loop */
  for(;;)
  {
    if (good_night_mode == 0){
    	mcp9600_read(&TEMP, T);
    	DATA.temp_td = (int16_t)(T[1]* 100);
    	DATA.temp_th = (int16_t)(T[0]* 100);
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

  printf("mounting SD card \n");
  mount_sd_card();
  res = open_file(FILE_NAME);

  /* Infinite loop */
  for(;;)
  {

	  tick = HAL_GetTick();

	  if (good_night_mode == 0){

		  res = write_to_file(&DATA, &buffer_size);
		  if (buffer_size > WRITE_BUFFER_SIZE){
			  turn_on(&STAT);
			  res = flush_buffer();
			  buffer_size = 0;
			  turn_off(&STAT);
		  }

		  line_counter ++;
	  	  if (line_counter > SECONDS_PER_FILE * 1000 / SAVE_INTERVAL){
			  buffer_size = 0;
			  line_counter = 0;
	  		  num_dat_file ++;
	  		  close_file();
	  		  sprintf(FILE_NAME,"FD%04u.BIN", num_dat_file);
	  		  res = open_file(FILE_NAME);
	  	  }

		  if (DEBUG_PRINT == 1) printf("tick: %ld \n",tick);
		  if (DEBUG_PRINT == 1) printf("flight phase: %u \n",flight_phase);
		  if (DEBUG_PRINT == 1) printf("T_a: %4.2f C \n",a[0]);
		  if (DEBUG_PRINT == 1) printf("ax: %4.2f m/s2 \n",a[1]);
		  if (DEBUG_PRINT == 1) printf("ay: %4.2f m/s2 \n",a[2]);
		  if (DEBUG_PRINT == 1) printf("az: %4.2f m/s2 \n",a[3]);
		  if (DEBUG_PRINT == 1) printf("p1: %4.2f mBar \n",p1);
		  if (DEBUG_PRINT == 1) printf("p2: %4.2f mBar \n",p2);
		  if (DEBUG_PRINT == 1) printf("p_t1: %4.2f C \n",t1);
		  if (DEBUG_PRINT == 1) printf("p_t2: %4.2f C \n",t2);
		  if (DEBUG_PRINT == 1) printf("T_H: %4.2f C \n",T[0]);
		  if (DEBUG_PRINT == 1) printf("T_D: %4.2f C \n",T[1]);
		  if (DEBUG_PRINT == 1) printf("T_C: %4.2f C \n",T[2]);
	  }

	  osDelay(SAVE_INTERVAL);
  }
  /* USER CODE END StartSDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

