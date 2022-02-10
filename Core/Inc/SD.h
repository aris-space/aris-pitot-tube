/* *
 *  Created on: Sep 14, 2020
 *  Author: ARIS / Linus Stoeckli
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "fatfs.h"
#include "utils.h"
#include "main.h"

#define WRITE_BUFFER_SIZE 1024
#define SECONDS_PER_FILE 10

FRESULT fresult;  // to store the result
UINT br, bw;   // file read/write count

FATFS fs;  // file system
FIL data_file;  // file
FIL fake_file;  // file
FIL log_file;  // file

void mount_sd_card();
void remount_sd_card();
FRESULT get_file_numbers(uint16_t *cnt1, uint16_t *cnt2);
FRESULT find_next_file_name(char *file_name);
FRESULT open_file(char *file_name);
FRESULT close_file(void);
FRESULT write_log_file(char *file_name, log_t * log, uint16_t * buffer_size);
FRESULT write_to_file(data_t * data, uint16_t * buffer_size);
FRESULT flush_buffer();



#endif /* INC_SD_H_ */
