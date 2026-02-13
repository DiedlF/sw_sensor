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

void flexible_log_file_implementation_t::wrap_around ( void)
{
  if( write_pointer > buffer_end)
    {
      unsigned part_length = write_pointer - buffer_end;
      uint32_t *to = buffer;
      uint32_t *from = buffer_end;
      while( part_length--)
	*to++ = *from++;

      write_pointer = to;
    }
}

bool flexible_log_file_implementation_t::sync_file( void)
{
  FRESULT fresult;
  fresult = f_sync (&out_file);
  return(fresult == FR_OK);
}

bool flexible_log_file_implementation_t::flush_buffer( void)
{
  if( write_pointer < buffer)
    return true; // still waiting ...

  UINT writtenBytes = 0;
  unsigned size_bytes = (buffer_end - buffer) * sizeof( uint32_t);
  f_write( &out_file, (const char *)buffer, size_bytes, &writtenBytes);

  wrap_around();

  return ( size_bytes == writtenBytes);
}

bool flexible_log_file_implementation_t::write_block (uint32_t *p_data, uint32_t size_words)
{
  ASSERT( write_pointer + size_words < buffer_absolute_limit);

  while( size_words --)
	*write_pointer++ = *p_data++;

  if( write_pointer > buffer_end)
    signal();
  return true;
}
