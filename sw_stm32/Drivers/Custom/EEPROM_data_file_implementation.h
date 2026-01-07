#ifndef CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_
#define CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_

#include "stdint.h"

typedef struct
{
  uint32_t * dest;
  uint32_t * source;
  unsigned n_words;
} flash_write_order;

void recover_and_initialize_flash( void);

#endif /* CUSTOM_EEPROM_DATA_FILE_IMPLEMENTATION_H_ */
