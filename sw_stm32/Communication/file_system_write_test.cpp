#include "system_configuration.h"
#include "FreeRTOS_wrapper.h"
#include "my_assert.h"
#include "EEPROM_data_file_implementation.h"
#include "common.h"

#if RUN_FLASH_WRITE_TESTER

uint64_t getTime_usec(void);

COMMON unsigned write_test_counter;

static void runnable( void *)
{
  bool success;
  recover_and_initialize_flash();
  uint64_t time;

  uint8_t loop_count;
  success = permanent_data_file.retrieve_data(0xfe, loop_count);
  if( not success)
    loop_count = 0;

  if( loop_count == 5)
    suspend();

  for( write_test_counter=0; write_test_counter < 9000; ++write_test_counter)
  {
    time = getTime_usec();
    success = permanent_data_file.store_data ( 0xa5, 2, &time);
    ASSERT( success);
  }

  ++loop_count;
  success = permanent_data_file.store_data ( 0xa5, loop_count);
  delay(100);
  ASSERT ( 0);
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

#endif
