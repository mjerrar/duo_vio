/*
 * Mutex.h
 *
 *  Created on: Mar 7, 2014
 *      Author: tpetri
 */

#ifndef MUTEX_H_
#define MUTEX_H_

namespace ait
{

class Condition;

class Mutex
{
friend Condition;
public:
	Mutex();
	~Mutex();

	//Acquire Lock
	void Lock() const;
	//Release Lock
	void Unlock() const;

protected:
	void *handle;			//this will be allocated in the respective implementations for win32 and posix
};


class Condition
{
public:
	Condition(const Mutex &mutex);
	~Condition();

	//Wait for condition variable to be notified. Return 'true' if we got the condition successfully.
	bool Wait() const;

	//Timed wait (relative time in seconds and microseconds -- 1 second = 1000000 microseconds). Return 'true' if we got the condition successfully.
	bool WaitTimed(unsigned wait_secs, unsigned wait_microsecs = 0) const;

	//Wake up all threads waiting for the condition variable
	void NotifyAll() const;

	//Wake up one thread waiting for the condition variable
	void NotifyOne() const;

protected:
	const Mutex &mutex;
	void *handle;
};

}

#endif /* MUTEX_H_ */
