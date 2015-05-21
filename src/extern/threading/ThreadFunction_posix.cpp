#include "Thread.h"

#include <pthread.h>
#include <stdlib.h>
#include <time.h>

template <class T, void * (T::* thread)(void *arg)>
ThreadFunction<T, thread>::ThreadFunction(T *obj, void *arg)
: obj_(obj)
, arg_(arg)
, handle(0)
, handle_attr(0)
{
	if (!handle) handle = (void *) new pthread_t;
	if (!handle_attr) handle_attr = (void *) new pthread_attr_t;
    pthread_attr_init((pthread_attr_t *)handle_attr);
    pthread_attr_setdetachstate((pthread_attr_t *)handle_attr, PTHREAD_CREATE_JOINABLE);
}

template <class T, void * (T::* thread)(void *arg)>
ThreadFunction<T, thread>::~ThreadFunction()
{
	Join();

	if (handle)
		delete (pthread_t *) handle;
	if (handle_attr)
		delete (pthread_attr_t *) handle_attr;
}

template <class T, void * (T::* thread)(void *arg)>
int ThreadFunction<T, thread>::Run()
{
	return pthread_create((pthread_t *) handle, (pthread_attr_t *) handle_attr, ThreadFunction<T, thread>::launchPad, this);
}


template <class T, void * (T::* thread)(void *arg)>
int ThreadFunction<T, thread>::Join()
{
	return pthread_join((pthread_t *) handle, NULL);
}

template <class T, void * (T::* thread)(void *arg)>
int ThreadFunction<T, thread>::SetLowPriority()
{
		int policy = 0;
		pthread_attr_getschedpolicy((pthread_attr_t *) handle_attr, &policy);
		int min_prio = sched_get_priority_min(policy);
		return pthread_setschedprio((pthread_t *) handle, min_prio);
}


template <class T, void * (T::* thread)(void *arg)>
void *ThreadFunction<T, thread>::launchPad(void *obj)
{
	ThreadFunction<T, thread>* t = reinterpret_cast<ThreadFunction<T, thread> *>(obj);
	return t->launch();
}

template <class T, void * (T::* thread)(void *arg)>
void *ThreadFunction<T, thread>::launch()
{
	return (obj_->*thread)(arg_);
}



ThreadMutex::ThreadMutex()
{
    handle = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init((pthread_mutex_t *) handle, NULL);
}

ThreadMutex::~ThreadMutex()
{
	if (handle)
	{
		pthread_mutex_destroy((pthread_mutex_t *) handle);
		free(handle);
		handle = 0;
	}
}

bool ThreadMutex::Lock()
{
	if (handle)
	{
		pthread_mutex_lock((pthread_mutex_t *) handle);
		return true;
	}
	return false;
}

bool ThreadMutex::TryLock()
{
	if (handle)
	{
		pthread_mutex_trylock((pthread_mutex_t *) handle);
		return true;
	}
	return false;
}

bool ThreadMutex::Unlock()
{
	if (handle)
	{
		pthread_mutex_unlock((pthread_mutex_t *) handle);
		return true;
	}
	return false;
}




ThreadCondition::ThreadCondition(const ThreadMutex &mutex) : mutex(mutex)
{
	handle = 0;
	handle = malloc(sizeof(pthread_cond_t));
	pthread_cond_init((pthread_cond_t *) handle, NULL);
}

ThreadCondition::~ThreadCondition()
{
	if (handle)
	{
		pthread_cond_destroy((pthread_cond_t *) handle);
		free(handle);
		handle = 0;
	}
}

bool ThreadCondition::Wait()
{
	if(!mutex.handle || !handle)
		return false;

	int ret = pthread_cond_wait((pthread_cond_t *) handle, (pthread_mutex_t *) mutex.handle);

	return ret == 0;
}

//Timed wait (seconds and microseconds)
bool ThreadCondition::WaitTimed(unsigned wait_secs, unsigned wait_microsecs)
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
void ThreadCondition::NotifyAll()
{
	if (handle)
		pthread_cond_broadcast((pthread_cond_t *) handle);
}

//Wake up one thread waiting for the condition variable
void ThreadCondition::NotifyOne()
{
	if (handle)
		pthread_cond_signal((pthread_cond_t *) handle);
}
