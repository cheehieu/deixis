// preprocessor directives
#ifndef MUTEX_H
#define MUTEX_H
#include <pthread.h>



// "Mutex" type definition
class Mutex
{
   public:

      // <constructors>
      Mutex();
      Mutex(const Mutex &mutex);

      // <destructors>
      ~Mutex();

      // <public utility functions>
      bool tryLock();
      bool lock();
      bool unlock();

   protected:

      // <protected data members>
      pthread_mutex_t handle_;
}; // Mutex

#endif

