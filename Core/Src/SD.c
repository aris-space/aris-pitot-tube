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
FIL Data_File;
FIL Cal_File;
FIL Log_File;

uint16_t bufsize(char *buf) {
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

void bufclear(char *buffer)  // clear buffer
{
	for (int i = 0; i < BUFLEN; i++) {
		buffer[i] = '\0';
	}
}

void mount_sd_card() {
	FRESULT res;
	do {
		res = f_mount(&Fat_Fs, "", 1);
		if (res != FR_OK) {
			if (DEBUG_PRINT == 1)
				printf("[STORAGE TASK] Failed mounting SD card: %d\n", res);
			osDelay(10);
		} else {
			if (DEBUG_PRINT == 1)
				printf("SD card mounted \n");
		}
	} while (res != FR_OK);
}

void unmount_sd_card() {
	FRESULT res;
	res = f_mount(0, "", 1);
	if (res != FR_OK) {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] Failed unmounting SD card: %d\n", res);
	} else {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] SD card unmounted \n");
	}
}

void remount_sd_card() {
	FRESULT res;
	close_file();
	do {
		HAL_SD_DeInit(&hsd);
		MX_SDIO_SD_Init();
		osDelay(50);
		MX_FATFS_Init();
		osDelay(50);
		memset(&Fat_Fs, 0, sizeof(FATFS));
		res = f_mount(&Fat_Fs, "", 1);
		if (res != FR_OK) {
			if (DEBUG_PRINT == 1)
				printf("[STORAGE TASK] Failed remounting SD card: %d\n", res);
			// force sd card to be reinitialized
			//HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			osDelay(10);
		} else {
			if (DEBUG_PRINT == 1)
				printf("SD card remounted \n");
		}
	} while (res != FR_OK);
}

FRESULT setup_dir(uint16_t *num_dir) {
	DIR dirs;
	char *fn;
	FILINFO Finfo;
	FRESULT res;

	*num_dir = 0;
	char dir_path[10];

	if ((res = f_opendir(&dirs, SDPath)) == FR_OK) {
		while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			fn = Finfo.fname;
			if (_FS_RPATH && fn[0] == '.')
				continue;
			if ((fn[0] == 'D') & (fn[1] == 'A') & (fn[2] == 'T')) {
				if (DEBUG_PRINT == 1)
					printf("found flight log folder: %s \n", fn);
				(*num_dir)++;
			}
		}
	}
	(*num_dir)++;
	sprintf(dir_path, "DAT%04u", (unsigned int) *num_dir);
	printf("creating new folder with name: %s OK.\n", dir_path);
	res = f_mkdir(dir_path);
	if (res == FR_OK) {
		printf("created new data folder.\n");
		return FR_OK;
	} else {
		printf("failed to create new data folder err = %d \n", res);
		return res;
	}

}

FRESULT get_file_numbers(uint16_t *cnt1, uint16_t *cnt2) {
	DIR dirs;
	char *fn;
	FILINFO Finfo;
	FRESULT res;

	*cnt1 = 0;
	*cnt2 = 0;

	if ((res = f_opendir(&dirs, SDPath)) == FR_OK) {
		while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {
			fn = Finfo.fname;
			if (_FS_RPATH && fn[0] == '.')
				continue;

			if ((fn[0] == 'F') & (fn[1] == 'D')) {
				if (DEBUG_PRINT == 1)
					printf("found flight log: %s \n", fn);
				(*cnt1)++;
			} else if ((fn[0] == 'L') & (fn[1] == 'O')) {
				if (DEBUG_PRINT == 1)
					printf("found log file: %s \n", fn);
				(*cnt2)++;
			}

		}
	}
	if (DEBUG_PRINT == 1)
		printf("\n this is the %hu th flight. \n", *cnt1);
	if (DEBUG_PRINT == 1)
		printf("\n this is the %hu th log file. \n", *cnt2);
	return res;
}

FRESULT open_file(char *file_name) {
	FRESULT res;
	if (DEBUG_PRINT == 1)
		printf("[STORAGE TASK] Opening data file\n");
	res = f_open(&Data_File, file_name, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] Failed opening data file \"%s\": %d\n",
					file_name, res);
		return res;
	} else {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] done, opened file.\n");
	}
	return res;
}

FRESULT close_file(void) {
	FRESULT res;
	if (DEBUG_PRINT == 1)
		printf("[STORAGE TASK] Closing data file\n");
	res = f_close(&Data_File);
	if (res != FR_OK) {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] Failed closing data file: %d\n", res);
		return res;
	} else {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] done, opened file.\n");
	}
	return res;
}

FRESULT init_log(char *file_name) {
	char msg[128];
	FRESULT res;

	sprintf(msg, "t = %lu, successfully created log file\n", HAL_GetTick());
	res = f_open(&Log_File, file_name, FA_OPEN_APPEND | FA_WRITE);
	// write the string to the file
	res = f_puts(msg, &Log_File);
	f_close(&Log_File);

	return res;
}

FRESULT log_msg(char *file_name, char *message) {
	FRESULT res;

	// Open the file with write access
	res = f_open(&Log_File, file_name, FA_OPEN_APPEND | FA_WRITE);

	// Move to offset to the end of the file
	res = f_lseek((&Log_File), f_size(&Log_File));

	// write the string to the file
	res = f_puts(message, &Log_File);

	// Close file
	f_close(&Log_File);

	bufclear(message);

	return res;
}

FRESULT write_cal_file(char *file_name, cal_t *cal, uint16_t *buffer_size) {
	FRESULT res;
	UINT bc; /* Data read/write count */
	if (DEBUG_PRINT == 1)
		printf("[STORAGE TASK] saving data to log file\n");
	bc = 0;

	res = f_open(&Cal_File, file_name, FA_OPEN_APPEND | FA_WRITE);

	res += f_write(&Cal_File, &(cal->baro1_cal_1), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro1_cal_2), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro1_cal_3), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro1_cal_4), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro1_cal_5), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro1_cal_6), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro2_cal_1), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro2_cal_2), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro2_cal_3), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro2_cal_4), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro2_cal_5), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->baro2_cal_6), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->accel_sens), 2, &bc);
	*buffer_size += bc;
	res += f_write(&Cal_File, &(cal->gyro_sens), 2, &bc);
	*buffer_size += bc;

	res += f_close(&Cal_File);
	if (res != FR_OK) {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] Failed saving log: %d\n", res);
		return res;
	} else {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] done, wrote %d bytes.\n", bc);
	}
	return res;
}

FRESULT write_to_file(data_t *data, uint16_t *buffer_size) {
	FRESULT res;
	UINT bc; /* Data read/write count */
	if (DEBUG_PRINT == 1)
		printf("[STORAGE TASK] saving to data file\n");
	bc = 0;
	res = f_write(&Data_File, &(data->tick), 4, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->flight_phase), 1, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->accel_x), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->accel_y), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->accel_z), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->gyro_x), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->gyro_y), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->gyro_z), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->accel_t), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->accel_ok), 1, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->baro1_D1), 4, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->baro1_D2), 4, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->baro1_ok), 1, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->baro2_D1), 4, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->baro2_D2), 4, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->baro2_ok), 1, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->temp_tc), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->temp_th), 2, &bc);
	*buffer_size += bc;
	res = f_write(&Data_File, &(data->temp_ok), 1, &bc);
	*buffer_size += bc;

	*buffer_size += bc;
	if (res != FR_OK) {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] Failed saving data: %d\n", res);
		return res;
	} else {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] done, wrote %u bytes.\n",
					(unsigned int) *buffer_size);
	}
	return res;
}

FRESULT flush_buffer() {
	FRESULT res;
	if (DEBUG_PRINT == 1)
		printf("[STORAGE TASK] syncing file\n");
	res = f_sync(&Data_File);
	if (res != FR_OK) {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] Failed syncing file: %d\n", res);
		return res;
	} else {
		if (DEBUG_PRINT == 1)
			printf("[STORAGE TASK] done, synced file.\n");
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
