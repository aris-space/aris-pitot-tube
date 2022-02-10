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
    	if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed mounting SD card: %d\n", res);
      // force sd card to be reinitialized
      //HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      osDelay(10);
    } else {
    	if (DEBUG_PRINT == 1) printf("SD card mounted \n");
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
    	if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed remounting SD card: %d\n", res);
      // force sd card to be reinitialized
      //HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      osDelay(10);
    } else {
    	if (DEBUG_PRINT == 1) printf("SD card remounted \n");
    }
  } while (res != FR_OK);
}

FRESULT get_file_numbers(uint16_t *cnt1, uint16_t *cnt2){
	DIR dirs;
	char *fn;
	FILINFO Finfo;

	*cnt1 = 0;
	*cnt2 = 0;

	if ((fresult = f_opendir(&dirs, SDPath)) == FR_OK)
	{
		while (((fresult = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0])
		{
			fn = Finfo.fname;
			if (_FS_RPATH && fn[0] == '.') continue;

			if ((fn[0] == 'F') & (fn[1] == 'L'))
			{
				if (DEBUG_PRINT == 1) printf("found flight log: %s \n",fn);
				*cnt1 ++;
			} else if ((fn[0] == 'L') & (fn[1] == 'O'))
			{
				if (DEBUG_PRINT == 1) printf("found log file: %s \n",fn);
				*cnt2 ++;
			}

		}
	}
	if (DEBUG_PRINT == 1) printf("\n this is the %hu th flight. \n", *cnt1);
	if (DEBUG_PRINT == 1) printf("\n this is the %hu th log file. \n", *cnt2);
}

FRESULT find_next_file_name(char *file_name) {
  FRESULT res;
  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Creating file name\n");
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

    if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Using file name: %s\n", file_name);

    res = f_closedir(&dj);
    if (res != FR_OK) {
    	if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed closing directory: %d\n", res);
    }
  } else {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed finding first or next file: %d\n", res);
  }

  return res;
}

FRESULT open_file(char *file_name) {
  FRESULT res;
  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Opening data file\n");
  res = f_open(&File, file_name, FA_OPEN_APPEND | FA_WRITE);
  if (res != FR_OK) {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed opening data file \"%s\": %d\n", file_name,
             res);
    return res;
  } else {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] done, opened file.\n");
  }
  return res;
}

FRESULT close_file(void) {
  FRESULT res;
  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Closing data file\n");
  res = f_close(&File);
  if (res != FR_OK) {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed closing data file: %d\n",
             res);
    return res;
  } else {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] done, opened file.\n");
  }
  return res;
}


FRESULT write_log_file(char *file_name, log_t * log, uint16_t * buffer_size) {
  FRESULT res;
  UINT bc;         /* Data read/write count */
  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] saving data to log file\n");
  bc = 0;

  res = f_open(&File, file_name, FA_OPEN_APPEND | FA_WRITE);

  res += f_write(&File, &log->file_number, 16, &bc);
  res += f_write(&File, &log->baro1_cal_1, 16, &bc);
  res += f_write(&File, &log->baro1_cal_2, 16, &bc);
  res += f_write(&File, &log->baro1_cal_3, 16, &bc);
  res += f_write(&File, &log->baro1_cal_4, 16, &bc);
  res += f_write(&File, &log->baro1_cal_5, 16, &bc);
  res += f_write(&File, &log->baro1_cal_6, 16, &bc);
  res += f_write(&File, &log->baro2_cal_1, 16, &bc);
  res += f_write(&File, &log->baro2_cal_2, 16, &bc);
  res += f_write(&File, &log->baro2_cal_3, 16, &bc);
  res += f_write(&File, &log->baro2_cal_4, 16, &bc);
  res += f_write(&File, &log->baro2_cal_5, 16, &bc);
  res += f_write(&File, &log->baro2_cal_6, 16, &bc);
  res += f_write(&File, &log->accel_sens, 16, &bc);
  res += f_write(&File, &log->gyro_sens, 16, &bc);

  res += f_close(&File);
  if (res != FR_OK) {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed saving log: %d\n", res);
      return res;
  } else {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] done, wrote %d bytes.\n", bc);
  }
  *buffer_size += bc;
  return res;
}

FRESULT write_to_file(data_t * data, uint16_t * buffer_size) {
  FRESULT res;
  UINT bc;         /* Data read/write count */
  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] saving data to file\n");
  bc = 0;
  res = f_write(&File, &data->tick, 32, &bc);
  res = f_write(&File, &data->accel_x, 16, &bc);
  res = f_write(&File, &data->accel_y, 16, &bc);
  res = f_write(&File, &data->accel_z, 16, &bc);
  res = f_write(&File, &data->accel_t, 16, &bc);
  res = f_write(&File, &data->baro1_D1, 32, &bc);
  res = f_write(&File, &data->baro1_D2, 32, &bc);
  res = f_write(&File, &data->baro2_D1, 32, &bc);
  res = f_write(&File, &data->baro2_D2, 32, &bc);
  res = f_write(&File, &data->temp_td, 16, &bc);
  res = f_write(&File, &data->temp_th, 16, &bc);
  if (res != FR_OK) {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed saving data: %d\n", res);
      return res;
  } else {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] done, wrote %d bytes.\n", bc);
  }
  *buffer_size += bc;
  return res;
}


FRESULT flush_buffer(){
  FRESULT res;
  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] syncing file\n");
  res = f_sync(&File);
  if (res != FR_OK) {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed syncing file: %d\n",
		   res);
	  return res;
  } else {
	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] done, synced file.\n");
  }
//  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] closing file \n");
//  res = f_close(&File);
//
//  if (res != FR_OK) {
//	  if (DEBUG_PRINT == 1) printf("[STORAGE TASK] Failed closing file \"%s\": %d\n", file_name,
//		 res);
//  return res;
//  }
  return res;
}
