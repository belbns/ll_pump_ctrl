#ifndef _MFLASH_H_
#define _MFLASH_H_

#include <stdint.h>

#define PAGE_15_ADDR		0x08003C00  // upper page of the flash memory
#define PARAMETERS_PAGE		PAGE_15_ADDR
#define CODES_SIGNATURE		0x020B

void save_parameters_to_flash(uint16_t *data_for_save, uint16_t data_length);

#endif
