#include "embedded_memory.h"
#include "embedded_math.h"
#include "common.h"
#include "FreeRTOS_wrapper.h"
#include "system_configuration.h"
#include "persistent_data_file.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "stm32f4xx_hal.h"

#define PAGE_0_HEAD ((uint32_t *)0x080C0000)
#define PAGE_1_HEAD ((uint32_t *)0x080E0000)
#define PAGE_SIZE_BYTES 0x20000
#define PAGE_SIZE_WORDS 0x08000
#define PAGE_SIZE_LONG_WORDS 0x04000

COMMON Queue <flash_write_order> flash_command_queue( 3);

COMMON EEPROM_file_system permanent_data_file;
extern Queue <flash_write_order> flash_command_queue;

void FLASH_write( uint32_t * dest, uint32_t * source, unsigned n_words)
{
  flash_write_order cmd;
  cmd.dest=dest;
  cmd.source=source;
  cmd.n_words=n_words;

  bool result = flash_command_queue.send( cmd, FLASH_ACCESS_TIMEOUT);
  ASSERT( result);
}

//!< order sector erase
bool erase_sector( unsigned sector)
{
  if( (sector != 0) && (sector != 1))
    return false;

  flash_write_order cmd;
  cmd.dest = sector == 0 ? PAGE_0_HEAD : PAGE_1_HEAD;
  cmd.source=0;
  cmd.n_words=PAGE_SIZE_WORDS;

  bool result = flash_command_queue.send( cmd, FLASH_ACCESS_TIMEOUT);
  ASSERT( result);
  return true;
}

//!< execute sector erase
bool erase_sector_operation( unsigned sector)
{
  uint64_t * location = sector == 0 ? (uint64_t *)PAGE_0_HEAD : (uint64_t *)PAGE_1_HEAD;
  unsigned size = PAGE_SIZE_LONG_WORDS;

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
  if ( permanent_data_file.in_use())
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
  ASSERT (permanent_data_file.in_use());
  bool success = permanent_data_file.store_data( id, 1, &value);
  if( not success)
    {
      file_system_page_swap();
      return not permanent_data_file.store_data( id, 1, &value);
    }
  return false; // = OK
}

bool read_EEPROM_value (EEPROM_PARAMETER_ID id, float &value)
{
  if (permanent_data_file.in_use())
    {
      return not permanent_data_file.retrieve_data( id, 1, &value);
    }
  else // legacy version
    {
      uint16_t data;
      if (HAL_OK != EE_ReadVariable( id, (uint16_t*) &data))
	return true;
      return (EEPROM_convert (id, (EEPROM_data_t&) data, value, READ));
    }
}

bool import_raw_EEPROM_data( EEPROM_PARAMETER_ID id, uint32_t * flash_address, unsigned size_words, uint16_t &datum)
{
  if( *(uint16_t *)flash_address == 0xEEEE) // dirty flash segment
    return false;

  uint16_t candidate;
  bool found = false;
  while( size_words --)
    {
      if( *flash_address == 0xffffffff) // erased flash, end of data range
	break;

      if( (*flash_address >> 16) == id)
	{
	candidate = (uint16_t)(*flash_address);
	found = true;
	}
      ++flash_address;
    }
  if( found)
    {
      datum = candidate;
      return true;
    }
  else
    return false;
}

bool import_single_EEPROM_value( EEPROM_PARAMETER_ID id, uint32_t * flash_address, unsigned size_words, float &value)
{
  EEPROM_data_t raw_EEPROM_value;

  if( import_raw_EEPROM_data( id, flash_address, size_words, raw_EEPROM_value.u16))
    {
      if( not EEPROM_convert( id, raw_EEPROM_value, value , true))
	return true;
    }
  return false;
}

bool import_legacy_EEPROM_data( uint32_t * flash_address, unsigned size_words)
{
  bool result = true;
  float value;

  for( 	const persistent_data_t * parameter = PERSISTENT_DATA;
	parameter < PERSISTENT_DATA + PERSISTENT_DATA_ENTRIES;
	++parameter)
    {
	if( not import_single_EEPROM_value( parameter->id, flash_address, size_words, value))
	  value = parameter->default_value;

	result &= permanent_data_file.store_data ( parameter->id, 1, &value);
    }
  return result;
}

void recover_and_initialize_flash( void)
{
  if( *(uint16_t *)0x080F8000 == 0 && *(int64_t *)0x080E0000 == -1) // old flash layout
    {
      // prepare page 0 for data import
      erase_sector( 0);
      permanent_data_file.set_memory_area( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_BYTES);
      bool success = import_legacy_EEPROM_data( PAGE_0_HEAD, PAGE_SIZE_BYTES / sizeof( uint32_t));
      ASSERT( success); // we have erased the sector and prepared the file system, so this shall be OK
      erase_sector( 1); // now we clean the upper sector from the old data
      return; // job done
    }
  if( *(uint16_t *)PAGE_0_HEAD == 0) // new flash layout, using page 0
    {
      erase_sector( 1);
      permanent_data_file.set_memory_area( PAGE_1_HEAD, PAGE_0_HEAD+PAGE_SIZE_BYTES);
      bool success = import_legacy_EEPROM_data( PAGE_1_HEAD, PAGE_SIZE_BYTES / sizeof( uint32_t));
      ASSERT( success); // we have erased the sector and prepared the file system, so this shall be OK
      erase_sector( 0); // now we clean the upper sector from the old data
      return; // job done
    }
  if( *(uint16_t *)PAGE_1_HEAD == 0) // new flash layout, using page 0
    {
      erase_sector( 0);
      permanent_data_file.set_memory_area( PAGE_0_HEAD, PAGE_0_HEAD+PAGE_SIZE_BYTES);
      bool success = import_legacy_EEPROM_data( PAGE_0_HEAD, PAGE_SIZE_BYTES / sizeof( uint32_t));
      ASSERT( success); // we have erased the sector and prepared the file system, so this shall be OK
      erase_sector( 1); // now we clean the upper sector from the old data
      return; // job done
    }

  // we did not find any legacy data and no valid new file system
  // so we clean the complete set of sectors and start using sector 0
  create_virgin_flash_file_system( true);
}

static void EEPROM_writing_runnable( void *)
{
  flash_write_order order;
  while( true)
    {
      flash_command_queue.receive( order, INFINITE_WAIT);

      if( order.source == 0 && order.n_words == PAGE_SIZE_WORDS) // erase commmand
	{
	  if( order.dest == PAGE_0_HEAD)
	    erase_sector_operation( 0);
	  else if( order.dest == PAGE_1_HEAD)
	    erase_sector_operation( 1);
	  return;
	}

      HAL_StatusTypeDef status;
      status = HAL_FLASH_Unlock();
      ASSERT(HAL_OK == status);

      while( order.n_words --)
	{
	  status = HAL_FLASH_Program( TYPEPROGRAM_WORD, (uint32_t)(order.dest), *(order.source) );
	  ASSERT( status == HAL_OK);
	  ++order.dest;
	  ++order.source;
	}

      status = HAL_FLASH_Lock();
      ASSERT(HAL_OK == status);
    }
}

#define STACKSIZE 256
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  {
    EEPROM_writing_runnable,
    "EEPROM",
    STACKSIZE,
    0,
    EEPROM_WRITER_PRIORITY,
    stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { PAGE_0_HEAD, PAGE_SIZE_BYTES, portMPU_REGION_READ_WRITE },
      { 0, 0, 0 }
    }
  };

static RestrictedTask EEPROM_accessor( p);
