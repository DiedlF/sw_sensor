#include "flexible_file_format.h"
#include <flexible_log_file_implementation.h>
#include "CRC16.h"
#include "my_assert.h"

bool flexible_log_file_implementation_t::open (char *file_name)
{
  FRESULT fresult;
  fresult = f_open (&out_file, (const TCHAR*)file_name, FA_CREATE_ALWAYS | FA_WRITE);
  if( fresult == FR_OK)
    {
      file_is_open = true;
      return true;
    }
  return false;
}

bool flexible_log_file_implementation_t::close( void)
{
  UINT writtenBytes = 0;
  f_write( &out_file, (const char *)flexible_log_file_t::buffer, (flexible_log_file_t::write_pointer - flexible_log_file_t::buffer) * sizeof( uint32_t), &writtenBytes);
  f_close ( &out_file);
  file_is_open = false;
  return true;
}

bool flexible_log_file_implementation_t::sync_file( void)
{
  FRESULT fresult;
  fresult = f_sync (&out_file);
  return(fresult == FR_OK);
}

bool flexible_log_file_implementation_t::flush_buffer( void)
{
  uint32_t *start;
  UINT writtenBytes = 0;
  if( status & WRITING_LOW)
    {
      ASSERT( not( status & FILLING_LOW));
      start = buffer;
    }
  else if( ( status & WRITING_HIGH))
    {
      ASSERT( not( status & FILLING_HIGH));
      start = second_part;
    }

  unsigned size_bytes = (buffer_end - buffer) / 2 * sizeof( uint32_t);
  f_write( &out_file, (const char *)start, size_bytes, &writtenBytes);

  status &= ~(WRITING_LOW | WRITING_HIGH);

  return ( size_bytes == writtenBytes);
}

bool flexible_log_file_implementation_t::write_block (uint32_t *p_data, uint32_t size_words)
{
  while( size_words --)
    {
	*write_pointer++ = *p_data++;

	if( write_pointer > buffer_end)
	  {
	    ASSERT(not (status & WRITING_LOW) )
	    write_pointer = buffer;

	    status &= ~FILLING_HIGH;
	    status |= FILLING_LOW;
	    status |= WRITING_HIGH;

	    signal();
	  }
	else if( (status & FILLING_LOW) and (write_pointer >= second_part ))
	  {
	    ASSERT( not (status & WRITING_HIGH) );

	    status &= ~FILLING_LOW;
	    status |= FILLING_HIGH;
	    status |= WRITING_LOW;

	    signal();
	  }
    }

  if( write_pointer > buffer_end)
    signal();
  return true;
}
