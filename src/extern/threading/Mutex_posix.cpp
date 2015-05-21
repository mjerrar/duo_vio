#include "Mutex.h"
#include <pthread.h>
#include <stdlib.h>

namespace ait
{

Mutex::Mutex()
{
	handle = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init((pthread_mutex_t *) handle, 0);
}

Mutex::~Mutex()
{
	if (handle)
	{
		pthread_mutex_destroy((pthread_mutex_t *) handle);
		free(handle);
		handle = 0;
	}
}

void Mutex::Lock() const
{
	if (handle) pthread_mutex_lock((pthread_mutex_t *) handle);
}

void Mutex::Unlock() const
{
	if (handle) pthread_mutex_unlock((pthread_mutex_t *) handle);
}




Condition::Condition(const Mutex &mutex) : mutex(mutex)
{
	handle = 0;
	handle = malloc(sizeof(pthread_cond_t));
	pthread_cond_init((pthread_cond_t *) handle, 0);
}

Condition::~Condition()
{
	if (handle)
	{
		pthread_cond_destroy((pthread_cond_t *) handle);
		free(handle);
		handle = 0;
	}
}

bool Condition::Wait() const
{
	if(!mutex.handle || !handle)
		return false;

	int ret = pthread_cond_wait((pthread_cond_t *) handle, (pthread_mutex_t *) mutex.handle);

	return ret == 0;
}

//Timed wait (seconds and microseconds)
bool Condition::WaitTimed(unsigned wait_secs, unsigned wait_microsecs) const
{
	if(!mutex.handle || !handle)
		return false;

	//Geting time
	timespec time;
	clock_gettime(CLOCK_REALTIME, &time);
	time.tv_sec  += wait_secs;
	time.tv_nsec += wait_microsecs * 1000;

	int ret = pthread_cond_timedwait((pthread_cond_t *) handle, (pthread_mutex_t *) mutex.handle, &time);

	return ret == 0;
}

//Wake up all threads waiting for the condition variable
void Condition::NotifyAll() const
{
	if (handle)
		pthread_cond_broadcast((pthread_cond_t *) handle);
}

//Wake up one thread waiting for the condition variable
void Condition::NotifyOne() const
{
	if (handle)
		pthread_cond_signal((pthread_cond_t *) handle);
}

}
