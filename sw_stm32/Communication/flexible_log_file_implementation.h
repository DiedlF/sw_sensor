#ifndef FLEXIBLE_LOG_FILE_IMPLEMENTATION_H_
#define FLEXIBLE_LOG_FILE_IMPLEMENTATION_H_

#include "stdint.h"
#include "fatfs.h"
#include "flexible_log_file.h"
using namespace std;

class flexible_log_file_implementation_t : public flexible_log_file_t
{
public:

  flexible_log_file_implementation_t ( uint32_t * buf, unsigned size_words)
  : flexible_log_file_t( buf, size_words)
  {
  }

  virtual ~flexible_log_file_implementation_t()
  {
    close();
  }

  bool open( char * file_name) override;
  bool close( void) override;

private:
  bool write_block( uint32_t * begin, uint32_t size_words) override;
  FIL out_file;
};

#endif
