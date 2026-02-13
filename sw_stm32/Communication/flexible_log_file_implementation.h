#ifndef FLEXIBLE_LOG_FILE_IMPLEMENTATION_H_
#define FLEXIBLE_LOG_FILE_IMPLEMENTATION_H_

#include "stdint.h"
#include "fatfs.h"
#include "flexible_log_file.h"
using namespace std;

typedef void ( *FPTR)( void); // declare void -> void function pointer

class flexible_log_file_implementation_t : public flexible_log_file_t
{
public:

  flexible_log_file_implementation_t ( uint32_t * buf, unsigned size_words, unsigned limit_words, FPTR _signal)
  : flexible_log_file_t( buf, size_words),
    buffer_absolute_limit( flexible_log_file_t::buffer + limit_words),
    signal( _signal)
  {
  }

  virtual ~flexible_log_file_implementation_t()
  {
    close();
  }

  bool open( char * file_name) override;
  bool flush_buffer( void);
  bool sync_file( void);
  bool close( void) override;

private:
  bool write_block( uint32_t * begin, uint32_t size_words) override;
  void wrap_around( void);
  FIL out_file;
  uint32_t *buffer_absolute_limit; // length of bile buffer incl. reserve words
  FPTR signal;
};

#endif
