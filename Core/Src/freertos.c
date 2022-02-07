/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

MS5803 PITOT1 = BARO1_INIT();
MS5803 PITOT2 = BARO2_INIT();

MCP9600 TEMP = MCP_INIT();
LED STAT = STAT_INIT();
LED RDY = RDY_INIT();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t flight_phase = PAD_IDLE;

volatile float a[8];
volatile float p1 = 0;
volatile float p2 = 0;
volatile float t1 = 0;
volatile float t2 = 0;
volatile float T[4];

volatile char buffer[BUFLEN]; // to store data
volatile uint16_t num_dat_file = 0;
volatile uint16_t num_log_file = 0;
volatile uint8_t SD_state = 0;
volatile uint16_t counter = 0;

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

	ms5803_init(&PITOT1);
	ms5803_init(&PITOT2);

	icm20601_init(&IMU);
	mcp9600_init(&TEMP);

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
    osDelay(100);
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
	uint16_t counter = 0;
  /* Infinite loop */
  for(;;)
  {
	icm20601_read_data(&IMU, a);
    osDelay(10);

	counter ++;
	if (counter > 20) {
	  counter = 0;
	  turn_on(&RDY);
	  osDelay(100);
	  printf("setting IMU to WOI\n");
	  icm20601_standby(&IMU);
	  printf("byebye...\n");
	  turn_off(&RDY);


//	  for (;;){
//		  uint8_t r;
//		  icm20601_read_int(&IMU, &r);
//		  if (r > 1) printf(" interrupt: %d \n", r);
//		  osDelay(10);
//
//	  }
	  osDelay(100);
	  turn_off(&RDY);
	  turn_off(&STAT);

	  HAL_PWR_DisableSleepOnExit();
	  // HAL_PWR_EnterSTANDBYMode();
	  HAL_SuspendTick();

	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	  // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);

	  osDelay(1000);
	  printf("hello...\n");


	  icm20601_init(&IMU);

	}
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
	  osDelay(3);

  	  ms5803_prep_pressure(&PITOT1);
	  ms5803_prep_pressure(&PITOT2);

	  osDelay(3);

	  ms5803_read_pressure(&PITOT1);
	  ms5803_read_pressure(&PITOT2);

	  ms5803_convert(&PITOT1, &p1, &t1);
	  ms5803_convert(&PITOT2, &p2, &t2);

	  osDelay(20);
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
  /* Infinite loop */
  for(;;)
  {
	mcp9600_read(&TEMP, T);
    osDelay(100);
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
	printf("mounting SD card \n");
	mount_sd_card();
	open_file("data.log");
	uint16_t buffer_size = 0;
  /* Infinite loop */
  for(;;)
  {

	  tick = HAL_GetTick();
	  //write_to_file(tick, &buffer_size);
	  if (buffer_size > 1024){
		  //flush_buffer();
		  buffer_size = 0;
	  }




	  /*

	  bufclear(buffer);
	  sprintf(buffer,"%ld, %d ,%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f\n",
				tick, flight_phase,t1,t2,a[0],T[0],T[1],T[2],p1,p2,a[1],a[2],a[3],a[4],a[5],a[6]);
	  if (DEBUG_PRINT == 1) printf(buffer);
	  write_to_SD(FILE_NAME, buffer);

	  counter ++;
	  if (counter > 100){
		  counter = 0;
		  num_dat_file ++;
		  sprintf(FILE_NAME,"FL%04u.CSV", num_dat_file);
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
	   */
	  osDelay(10);
  }
  /* USER CODE END StartSDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

