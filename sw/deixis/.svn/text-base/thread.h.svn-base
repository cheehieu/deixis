// preprocessor directives
#ifndef THREAD_H
#define THREAD_H
#include <pthread.h>
#include <signal.h>



// type redefinitions
typedef void*        ThreadReturn;
typedef void*        ThreadParam;
typedef ThreadReturn (*ThreadFunc)(ThreadParam);



// "Thread" type definition
class Thread
{
   public:

      // <constructors>
      Thread(ThreadFunc  func    = NULL,
             ThreadParam param   = NULL,
             bool        running = false);
      Thread(const Thread &thread);

      // <destructors>
      ~Thread();

      // <public accessor functions>
      pthread_t getHandle();

      // <public utility functions>
      int  start();
      int  start(ThreadFunc func, ThreadParam param = NULL);
      int  stop();
      bool isRunning();

      // <public friend functions>
      int waitForThread(Thread &thread);

   protected:

      // <protected data members>
      pthread_t   handle_;
      ThreadFunc  func_;
      ThreadParam param_;
      bool        running_;
}; // Thread

#endif

