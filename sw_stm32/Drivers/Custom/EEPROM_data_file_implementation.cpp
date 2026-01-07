#include "embedded_memory.h"
#include "embedded_math.h"
#include "common.h"
#include "FreeRTOS_wrapper.h"
#include "system_configuration.h"
#include "persistent_data_file.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "stm32f4xx_hal.h"

#define PAGE_0_HEAD 0x080C0000
#define PAGE_1_HEAD 0x080E0000
#define PAGE_SIZE_BYTES 0x20000

COMMON bool using_permanent_data_file = false;
COMMON EEPROM_file_system permanent_data_file;
extern Queue <flash_write_order> flash_command_queue;

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words)
{
  flash_write_order cmd;
  cmd.dest=dest;
  cmd.source=source;
  cmd.n_words=n_words;

  bool result = flash_command_queue.send( cmd, 10);
  ASSERT( result);
}

bool setup_flash_file_system( void)
{
  if( *(__IO uint16_t*)PAGE_1_HEAD != 0xffffffff)
    permanent_data_file.set_memory_area( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_BYTES);
  else
    permanent_data_file.set_memory_area( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_BYTES);

  return permanent_data_file.is_consistent();
}

bool erase_sector( unsigned sector)
{
  uint64_t * location = sector == 0 ? (uint64_t *)PAGE_0_HEAD : (uint64_t *)PAGE_1_HEAD;
  unsigned size = PAGE_SIZE_BYTES / sizeof( uint64_t);

  bool is_erased = true;
  while( size --)
    {
    if( *location++ != 0xffffffffffffffff)
      {
	is_erased = false;
	break;
      }
    }

  if( is_erased)
    return true;

  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef EraseInit;
  uint32_t SectorError = 0;

  EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInit.VoltageRange = VOLTAGE_RANGE;

  EraseInit.Sector = sector == 0 ? FLASH_SECTOR_10 : FLASH_SECTOR_11;
  EraseInit.NbSectors = 1;
  status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
  return (status == HAL_OK);
}

bool create_virgin_flash_file_system( bool wipe_all = false)
{
  if( wipe_all)
    return erase_sector( 0) && erase_sector( 1);

  uint16_t header = *(__IO uint16_t*)PAGE_0_HEAD;
  if( header == 0) // page 0 marked valid
    {
      if( not erase_sector( 1))
	return false;

      permanent_data_file.set_memory_area( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_BYTES);
    }
  else
    {
      if( not erase_sector( 0))
	return false;

      permanent_data_file.set_memory_area( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_BYTES);
    }
  return true;
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
  else // using old EEPROM data
    {
      float value = 0.0f;
      read_EEPROM_value( id, value);
      return value;
    }
}

void file_system_page_swap( void)
{
  if( permanent_data_file.get_head() == (void *)PAGE_1_HEAD)
    {
      bool result = erase_sector( 0);
      ASSERT( result);
      EEPROM_file_system new_data_file( (EEPROM_file_system_node *)PAGE_0_HEAD, (EEPROM_file_system_node *)(PAGE_0_HEAD+PAGE_SIZE_BYTES));
      new_data_file.import_all_data(permanent_data_file);
      result = erase_sector( 1);
      ASSERT( result);
      permanent_data_file.set_memory_area( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_BYTES);
     }
  else
    {
      bool result = erase_sector( 1);
      ASSERT( result);
      EEPROM_file_system new_data_file( (EEPROM_file_system_node *)PAGE_1_HEAD, (EEPROM_file_system_node *)(PAGE_1_HEAD+PAGE_SIZE_BYTES));
      new_data_file.import_all_data(permanent_data_file);
      result = erase_sector( 0);
      ASSERT( result);
      permanent_data_file.set_memory_area( PAGE_1_HEAD, PAGE_1_HEAD+PAGE_SIZE_BYTES);
    }
}

bool write_EEPROM_value (EEPROM_PARAMETER_ID id, float value)
{
  if (using_permanent_data_file)
    {
      bool success = permanent_data_file.store_data( id, 1, &value);
      if( not success)
	{
	  file_system_page_swap();
	  return not permanent_data_file.store_data( id, 1, &value);
	}
      return false; // = OK
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
      return not permanent_data_file.retrieve_data( id, 1, &value);
    }
  else
    {
      uint16_t data;
      if (HAL_OK != EE_ReadVariableBuffered ( id, (uint16_t*) &data))
	return true;
      return (EEPROM_convert (id, (EEPROM_data_t&) data, value, READ));
    }
}

bool import_legacy_EEPROM_data( void)
{
  float value;
  bool result = true;

  value = configuration (SENS_TILT_ROLL);
  result &= permanent_data_file.store_data (SENS_TILT_ROLL, 1, &value);

  value = configuration (SENS_TILT_PITCH);
  result &= permanent_data_file.store_data (SENS_TILT_PITCH, 1, &value);

  value = configuration (SENS_TILT_YAW);
  result &= permanent_data_file.store_data (SENS_TILT_YAW, 1, &value);

  value = configuration (PITOT_OFFSET);
  result &= permanent_data_file.store_data (PITOT_OFFSET, 1, &value);

  value = configuration (PITOT_SPAN);
  result &= permanent_data_file.store_data (PITOT_SPAN, 1, &value);

  value = configuration (QNH_OFFSET);
  result &= permanent_data_file.store_data (QNH_OFFSET, 1, &value);

  value = configuration (VARIO_TC);
  result &= permanent_data_file.store_data (VARIO_TC, 1, &value);

  value = configuration (VARIO_INT_TC);
  result &= permanent_data_file.store_data (VARIO_INT_TC, 1, &value);

  value = configuration (VARIO_P_TC);
  result &= permanent_data_file.store_data (VARIO_P_TC, 1, &value);

  value = configuration (WIND_TC);
  result &= permanent_data_file.store_data (WIND_TC, 1, &value);

  value = configuration (MEAN_WIND_TC);
  result &= permanent_data_file.store_data (MEAN_WIND_TC, 1, &value);

  value = configuration (HORIZON);
  result &= permanent_data_file.store_data (HORIZON, 1, &value);

  value = configuration (GNSS_CONFIGURATION);
  result &= permanent_data_file.store_data (GNSS_CONFIGURATION, 1, &value);

  value = configuration (ANT_BASELENGTH);
  result &= permanent_data_file.store_data (ANT_BASELENGTH, 1, &value);

  value = configuration (ANT_SLAVE_DOWN);
  result &= permanent_data_file.store_data (ANT_SLAVE_DOWN, 1, &value);

  value = configuration (ANT_SLAVE_RIGHT);
  result &= permanent_data_file.store_data (ANT_SLAVE_RIGHT, 1, &value);

  return result;
}

