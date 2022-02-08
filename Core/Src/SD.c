/* *
 *  Created on: Sep 14, 2020
 *  Author: ARIS / Linus Stoeckli
 */

#include "SD.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "sdio.h"
#include <stdlib.h>     /* strtof */

FATFS Fat_Fs;
FIL File;


void mount_sd_card() {
  FRESULT res;
  do {
    res = f_mount(&Fat_Fs, "", 1);
    if (res != FR_OK) {
      printf("[STORAGE TASK] Failed mounting SD card: %d\n", res);
      // force sd card to be reinitialized
      //HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      osDelay(10);
    } else {
      printf("SD card mounted \n");
    }
  } while (res != FR_OK);
}

void remount_sd_card() {
  FRESULT res;
  // f_close(&EULER_LOG_FILE);
  do {
    HAL_SD_DeInit(&hsd);
    MX_SDIO_SD_Init();
    osDelay(50);
    MX_FATFS_Init();
    osDelay(50);
    memset(&Fat_Fs, 0, sizeof(FATFS));
    res = f_mount(&Fat_Fs, "", 1);
    if (res != FR_OK) {
      printf("[STORAGE TASK] Failed remounting SD card: %d\n", res);
      // force sd card to be reinitialized
      //HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      osDelay(10);
    } else {
      printf("SD card remounted \n");
    }
  } while (res != FR_OK);
}

FRESULT find_next_file_name(char *file_name) {
  FRESULT res;
  printf("[STORAGE TASK] Creating file name\n");
  uint16_t file_number = 1;

  DIR dj;
  FILINFO fno;
  res = f_findfirst(&dj, &fno, "", "LOG_???.CSV");
  while (res == FR_OK && fno.fname[0]) {
    uint16_t current_file_number = (fno.fname[4] - '0') * 100 +
                                   (fno.fname[5] - '0') * 10 +
                                   (fno.fname[6] - '0');
    if (current_file_number + 1 > file_number) {
      file_number = current_file_number + 1;
    }
    res = f_findnext(&dj, &fno);
    osDelay(10);
  }
  file_number = file_number <= 999 ? file_number : 999;

  if (res == FR_OK) {
    strcpy(file_name, "LOG_000.CSV");
    file_name[6] = '0' + file_number % 10;
    file_name[5] = '0' + (file_number / 10) % 10;
    file_name[4] = '0' + (file_number / 100) % 10;

    printf("[STORAGE TASK] Using file name: %s\n", file_name);

    res = f_closedir(&dj);
    if (res != FR_OK) {
    	printf("[STORAGE TASK] Failed closing directory: %d\n", res);
    }
  } else {
	  printf("[STORAGE TASK] Failed finding first or next file: %d\n", res);
  }

  return res;
}

FRESULT open_file(char *file_name) {
  FRESULT res;
  printf("[STORAGE TASK] Opening log file\n");
  res = f_open(&File, file_name, FA_OPEN_APPEND | FA_WRITE);
  if (res != FR_OK) {
	  printf("[STORAGE TASK] Failed opening log file \"%s\": %d\n", file_name,
             res);
    return res;
  } else {
	  printf("[STORAGE TASK] done, opened file.\n");
  }
  return res;
}
FRESULT write_to_file(uint32_t tick, uint16_t * buffer_size) {
  FRESULT res;
  UINT bc;         /* Data read/write count */
  int32_t data[3] = {0, 1, 31415};
  data[2] = (int32_t)tick;
  printf("[STORAGE TASK] saving data to file\n");
  bc = 0;
  res = f_write(&File, data, sizeof(data), &bc);
  if (res != FR_OK) {
	  printf("[STORAGE TASK] Failed saving data: %d\n",
             res);
      return res;
  } else {
	  printf("[STORAGE TASK] done, wrote %d bytes.\n", bc);
  }
  *buffer_size += bc;
  return res;
}


FRESULT flush_buffer(){
  FRESULT res;
  printf("[STORAGE TASK] syncing file\n");
  res = f_sync(&File);
  if (res != FR_OK) {
	  printf("[STORAGE TASK] Failed syncing file: %d\n",
		   res);
	  return res;
  } else {
	  printf("[STORAGE TASK] done, synced file.\n");
  }
//  printf("[STORAGE TASK] closing file \n");
//  res = f_close(&File);
//
//  if (res != FR_OK) {
//	  printf("[STORAGE TASK] Failed closing file \"%s\": %d\n", file_name,
//		 res);
//  return res;
//  }
  return res;
}
