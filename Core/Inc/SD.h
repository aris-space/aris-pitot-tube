/* *
 *  Created on: Sep 14, 2020
 *  Author: ARIS / Linus Stoeckli
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "fatfs.h"
#include "main.h"

FRESULT fresult;  // to store the result
UINT br, bw;   // file read/write count

FATFS fs;  // file system
FIL data_file;  // file
FIL fake_file;  // file
FIL log_file;  // file

void mount_sd_card();
void remount_sd_card();
FRESULT find_next_file_name(char *file_name);
FRESULT open_file(char *file_name);
FRESULT write_to_file(uint32_t tick, uint16_t * buffer_size);
FRESULT flush_buffer();



#endif /* INC_SD_H_ */
