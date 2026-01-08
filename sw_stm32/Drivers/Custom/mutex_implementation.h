#ifndef MUTEX_IMPLEMENTATION_H_
#define MUTEX_IMPLEMENTATION_H_

#include "FreeRTOS_wrapper.h"

extern Mutex EEPROM_lock;

class Mutex_Wrapper_Type
{
public:
  void lock( void)
  {
    EEPROM_lock.lock(10);
  }
  void unlock( void)
  {
    EEPROM_lock.release();
  }
};

#include "scoped_lock.h"

extern Mutex_Wrapper_Type my_mutex;
#define LOCK_SECTION() ScopedLock lock( my_mutex)

#endif /* MUTEX_IMPLEMENTATION_H_ */
