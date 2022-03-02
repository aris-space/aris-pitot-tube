/* *
 *  Created on: Sep 14, 2020
 *  Author: ARIS / Linus Stoeckli
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "fatfs.h"
#include "utils.h"
#include "main.h"

#define WRITE_BUFFER_SIZE 511
#define SECONDS_PER_FILE 10

extern FATFS Fat_Fs;
extern FIL Data_File;
extern FIL Cal_File;
extern FIL Log_File;

void mount_sd_card();
void remount_sd_card();
void unmount_sd_card();
FRESULT setup_dir(uint16_t *num_dir);
FRESULT get_file_numbers(uint16_t *cnt1, uint16_t *cnt2);
FRESULT open_file(char *file_name);
FRESULT close_file(void);
FRESULT init_log(char *file_name);
FRESULT log_msg(char *file_name, char *message);
FRESULT write_cal_file(char *file_name, cal_t *cal_container, uint16_t *buffer_size);
FRESULT write_to_file(data_t *data_container, uint16_t *buffer_size);
FRESULT flush_buffer();

#endif /* INC_SD_H_ */
