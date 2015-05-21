#include "Thread.h"
#include <pthread.h>

namespace ait
{

Thread::~Thread()
{
	if (IsRunning())
	{
		Stop();
		Join();
	}
	if (handle)
		delete (pthread_t *) handle;
}

void Thread::Start()
{
	//we allow only one instance of a thread
	if (mbRunning) return;

    mbStopping = false;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    if (!handle) handle = (void *) new pthread_t;
    pthread_create((pthread_t *)handle, &attr, threadproc, this);
}

void Thread::Join()
{
	if (handle) pthread_join(*(pthread_t *)handle, 0);
}

void* Thread::threadproc(void* param)
{
	Thread* thread = (Thread*) param;
//   pthread_setspecific(ourKey, thread);	//TODO: check for what is this actually really needed?
	thread->mbRunning = true;
	thread->Run();
	thread->mbRunning = false;
	pthread_exit(0);
	return 0;
}

}
