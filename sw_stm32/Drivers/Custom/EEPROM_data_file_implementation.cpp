#include "embedded_memory.h"
#include "embedded_math.h"
#include "common.h"
#include "FreeRTOS_wrapper.h"
#include "system_configuration.h"
#include "persistent_data_file.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "eeprom.h"

COMMON bool using_permanent_data_file = false;
COMMON EEPROM_file_system permanent_data_file( 0, 0);

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words)
{
  flash_write_order cmd;
  cmd.dest=dest;
  cmd.source=source;
  cmd.n_words=n_words;

  extern Queue <flash_write_order> flash_command_queue;
  bool result = flash_command_queue.send( cmd, 10);
  ASSERT( result);
}

flash_state_t check_flash_status( void)
{
  if( 0 )
    ;
}

float configuration (EEPROM_PARAMETER_ID id)
{
  if (using_permanent_data_file)
    {
      float value;
      // try to read float object size one
      bool result = permanent_data_file.retrieve_data (id, 1, (uint32_t *)&value);
      if (result)
	return value;
      else // read direct data ( 8 bit) object
	{
	  uint8_t value;
	  result = permanent_data_file.retrieve_data (id, value);
	  ASSERT( result);
	  return (float)value;
	}
    }
  else
    {
      float value = 0.0f;
      read_EEPROM_value( id, value);
      return value;
    }
}

bool write_EEPROM_value (EEPROM_PARAMETER_ID id, float value)
{
  if (using_permanent_data_file)
    {
      return true; // todo fixme
    }
  else
    {
      EEPROM_data_t EEPROM_value;
      if (EEPROM_convert (id, EEPROM_value, value, WRITE))
	return true; // error

      return EE_WriteVariableBuffered (id, EEPROM_value.u16);
    }
}

bool read_EEPROM_value (EEPROM_PARAMETER_ID id, float &value)
{
  if (using_permanent_data_file)
    {
      return true; // todo fixme
    }
  else
    {
      uint16_t data;
      if (HAL_OK != EE_ReadVariableBuffered (id, (uint16_t*) &data))
	return true;
      return (EEPROM_convert (id, (EEPROM_data_t&) data, value, READ));
    }
}
