#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "common.h"

uint64_t getTime_usec(void);

COMMON unsigned write_test_counter;

static void runnable( void *)
{
  bool success;
  delay( 2000);
  uint64_t time;
  while( true)
    for( write_test_counter=0; write_test_counter < 12000; ++write_test_counter)
    {
      time = getTime_usec();
      success = permanent_data_file.store_data ( 0xa5, 2, &time);
      if( not success)
        {
          file_system_page_swap();
          delay( 3000); // todo patch
          success = permanent_data_file.store_data( 0xa5, 2, &time);
          ASSERT( success);
        }
    }
}

#define STACKSIZE 128
static uint32_t __ALIGNED(STACKSIZE*sizeof(uint32_t)) stack_buffer[STACKSIZE];

static ROM TaskParameters_t p =
  {
  runnable,
  "WRITE_TEST",
  STACKSIZE,
  0,
  EEPROM_WRITER_PRIORITY,
  stack_buffer,
    {
      { COMMON_BLOCK, COMMON_SIZE,  portMPU_REGION_READ_WRITE },
      { (void *)0x080C0000, 0x00040000, portMPU_REGION_READ_WRITE}, // EEPROM
      { 0, 0, 0}
    }
  };

COMMON RestrictedTask file_write_tester (p);
