// preprocessor directives
#include "thread.h"



// <constructors>

//
Thread::Thread(ThreadFunc func, ThreadParam param, bool running)
   : handle_(0), func_(func), param_(param), running_(false)
{
   if (running) start();
}  // Thread(ThreadFun, ThreadParam, bool)



// <destructors>

//
Thread::~Thread()
{
   stop();
}  // ~Thread()



// <public accessor functions>

//
pthread_t Thread::getHandle()
{
   return handle_;
}  // getHandle()



// <public utility functions>

//
int Thread::start()
{
   int retVal = pthread_create(&handle_, NULL, func_, param_);
   return running_ = (retVal == 0);
}  // start()



//
int Thread::start(ThreadFunc func, ThreadParam param)
{
   func_  = func;
   param_ = param;
   return start();
}  // start(ThreadFunc, ThreadParam)



//
// NOTE: DOES NOT WORK!!! :-(
int Thread::stop()
{
   if (!isRunning()) return 0;
   int retVal = pthread_kill(handle_, 0);
   return running_ = !(retVal == 0);
}  // stop()



//
bool Thread::isRunning()
{
   return running_;
}  // isRunning()



//
int waitForThread(Thread &thread)
{
   if (!thread.isRunning()) return 0;
   return pthread_join(thread.getHandle(), NULL);
}  // waitForThread(Thread &thread)

