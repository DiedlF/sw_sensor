#include "flexible_file_format.h"
#include <flexible_log_file_implementation.h>
#include "CRC16.h"

bool flexible_log_file_stream_t::open (char *file_name)
{
  FRESULT fresult;
  fresult = f_open (&out_file, (const TCHAR*)file_name, FA_CREATE_ALWAYS | FA_WRITE);
  return (fresult == FR_OK);
}

bool flexible_log_file_stream_t::close( void)
{
  UINT writtenBytes = 0;
  f_write( &out_file, (const char *)flexible_log_file_t::buffer, (flexible_log_file_t::write_pointer - flexible_log_file_t::buffer) * sizeof( uint32_t), &writtenBytes);
  f_close ( &out_file);
  return true;
}

bool flexible_log_file_stream_t::write_block (uint32_t *p_data, uint32_t size_words)
{
  UINT writtenBytes = 0;
  if( write_pointer + size_words > buffer_end)
    {
      unsigned part_length = buffer_end - write_pointer;
      unsigned remaining_length = size_words - part_length;
      while( part_length --)
	*write_pointer++ = *p_data++;

      unsigned size_bytes = (buffer_end - buffer) * sizeof( uint32_t);
      f_write( &out_file, (const char *)buffer, size_bytes, &writtenBytes);
      if( size_bytes != writtenBytes)
	return false;

      write_pointer = buffer;
      while( remaining_length--)
	*write_pointer++ = *p_data++;
    }
  else
    {
      while( size_words --)
	*write_pointer++ = *p_data++;
    }
  return true;
}
