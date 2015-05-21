#include "Mutex.h"
#include <windows.h>

namespace ait
{

Mutex::Mutex()
{
	handle = (void *) new CRITICAL_SECTION;
	if (!InitializeCriticalSectionAndSpinCount((CRITICAL_SECTION *)handle, 0x00000400))
	{
		delete handle;
		handle = 0;
	}
}

Mutex::~Mutex()
{
	if (handle)
	{
		DeleteCriticalSection((CRITICAL_SECTION *)handle);
		delete handle;
		handle = 0;
	}
}

void Mutex::Lock() const
{
	if (handle) EnterCriticalSection((CRITICAL_SECTION *)handle);
}

void Mutex::Unlock() const
{
	if (handle) LeaveCriticalSection((CRITICAL_SECTION *)handle);
}




Condition::Condition(const Mutex &mutex) : mutex(mutex)
{
	handle = new CONDITION_VARIABLE;
	InitializeConditionVariable ((CONDITION_VARIABLE*)handle);
}

Condition::~Condition()
{
	if (handle)
	{
		delete handle;
		handle = 0;
	}
}

bool Condition::Wait() const
{
	return !SleepConditionVariableCS ((CONDITION_VARIABLE*)handle, (CRITICAL_SECTION*)mutex.handle, INFINITE);
}

//Timed wait (seconds and microseconds)
bool Condition::WaitTimed(unsigned wait_secs, unsigned wait_microsecs) const
{
	return !SleepConditionVariableCS ((CONDITION_VARIABLE*)handle, (CRITICAL_SECTION*)mutex.handle, wait_secs*1000 + wait_microsecs/1000);
}

//Wake up all threads waiting for the condition variable
void Condition::NotifyAll() const
{
	WakeAllConditionVariable ((CONDITION_VARIABLE*)handle);
}

//Wake up one thread waiting for the condition variable
void Condition::NotifyOne() const
{
	WakeConditionVariable ((CONDITION_VARIABLE*)handle);
}


}
