#include "Thread.h"
#include <process.h>
#include <windows.h>

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
		delete (uintptr_t *) handle;
}

void Thread::Start()
{
	//we allow only one instance of a thread
	if (mbRunning) return;

    mbStopping = false;
    if (!handle) handle = (void *) new uintptr_t;
	*((uintptr_t *)handle) = _beginthread(threadproc2, 0, this);
}

void Thread::Join()
{
	if (handle) WaitForSingleObject((HANDLE)*((uintptr_t *)handle), INFINITE);
}

void Thread::threadproc2(void* param)
{
	Thread* thread = (Thread*) param;
	thread->mbRunning = true;
	thread->Run();
	thread->mbRunning = false;
}

}
