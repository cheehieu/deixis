// preprocessor directives
#include "mutex.h"



// <constructors>

//
Mutex::Mutex()
{
   pthread_mutex_init(&handle_, NULL);
}  // Mutex()



//
Mutex::Mutex(const Mutex &mutex): handle_(mutex.handle_)
{
}  // Mutex(const Mutex &)



// <destructors>

//
Mutex::~Mutex()
{
   pthread_mutex_destroy(&handle_);
}  // ~Mutex()



// <public utility functions>

//
bool Mutex::tryLock()
{
   return !pthread_mutex_trylock(&handle_);
}  // tryLock()



//
bool Mutex::lock()
{
   return !pthread_mutex_lock(&handle_);
}  // lock()



//
bool Mutex::unlock()
{
   return !pthread_mutex_unlock(&handle_);
}  // unlock()

