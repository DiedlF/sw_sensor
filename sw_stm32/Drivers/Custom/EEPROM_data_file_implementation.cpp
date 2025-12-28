#include "embedded_memory.h"
#include "embedded_math.h"
#include "common.h"
#include "system_configuration.h"
#include "persistent_data_file.h"

COMMON bool using_permanent_data_file = false;
COMMON EEPROM_file_system permanent_data_file( 0, 0);

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words)
{
}






