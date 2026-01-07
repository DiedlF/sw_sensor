/*
 * EEPROM_data_file_implementation.h
 *
 *  Created on: Jan 6, 2026
 *      Author: schaefer
 */

#ifndef CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_
#define CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_

#include "stdint.h"

typedef struct
{
  uint32_t * dest;
  uint32_t * source;
  unsigned n_words;
} flash_write_order;

bool import_legacy_EEPROM_data( void);
bool setup_flash_file_system( void);

#endif /* CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_ */
