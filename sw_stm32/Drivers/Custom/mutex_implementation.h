#ifndef MUTEX_IMPLEMENTATION_H_
#define MUTEX_IMPLEMENTATION_H_

#include "FreeRTOS_wrapper.h"
#include "my_assert.h"

extern Mutex EEPROM_lock;

class Mutex_Wrapper_Type
{
public:
  Mutex_Wrapper_Type( void)
  : lock_count(0)
  {}

  void lock( void)
  {
    bool success;
    if( lock_count == 0)
      success = EEPROM_lock.lock(2000); // todo patch check time
    ASSERT( success);
    ++lock_count;
  }

  void unlock( void)
  {
    ASSERT( lock_count > 0);
        --lock_count;
    if( lock_count == 0)
      EEPROM_lock.release();
  }
private:
  unsigned lock_count;
};

#include "scoped_lock.h"

extern Mutex_Wrapper_Type my_mutex;
#define LOCK_SECTION() ScopedLock lock( my_mutex)

#endif /* MUTEX_IMPLEMENTATION_H_ */
