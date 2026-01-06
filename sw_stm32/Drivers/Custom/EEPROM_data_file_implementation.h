/*
 * EEPROM_data_file_implementation.h
 *
 *  Created on: Jan 6, 2026
 *      Author: schaefer
 */

#ifndef CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_
#define CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_

#include "stdint.h"

typedef enum {
  NO_EEPROM_DATA, // no data at all
  OLD_FORMAT_PAGE_080c0000,
  OLD_FORMAT_PAGE_080e0000,
  NEW_FORMAT_PAGE_080c0000,
  NEW_FORMAT_PAGE_080e0000,
  EEPROM_DATA_INVALID
} flash_state_t;

typedef struct
{
  uint32_t * dest;
  uint32_t * source;
  unsigned n_words;
} flash_write_order;

#endif /* CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_ */
