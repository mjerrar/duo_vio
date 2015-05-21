/*
 * ThreadFunction.h
 *
 *  Created on: Mar 7, 2014
 *      Author: thomasgubler
 */

#ifndef THREAD_FUNCTION_H_
#define THREAD_FUNCTION_H_

template <class T, void * (T::* thread)(void *arg)>
class ThreadFunction
{

public:
	ThreadFunction(T *obj, void *arg);
	~ThreadFunction();

public:
	int Run();
	int Join();
	int SetLowPriority();

protected:
	void *launch();
	static void *launchPad(void *obj);

protected:
    T *obj_;
    void *arg_;
    void *handle;
    void *handle_attr;
};

class ThreadCondition;

class ThreadMutex {
friend ThreadCondition;
public:
	ThreadMutex();
	virtual ~ThreadMutex();

public:
	virtual bool Lock();
	virtual bool TryLock();
	virtual bool Unlock();

protected:
	void *handle;
};

class ThreadCondition
{
public:
	ThreadCondition(const ThreadMutex &mutex);
	virtual ~ThreadCondition();

public:
	bool Wait();
	bool WaitTimed(unsigned int wait_secs, unsigned int wait_microsecs = 0);
	void NotifyOne();
	void NotifyAll();

protected:
	const ThreadMutex &mutex;
	void *handle;
};

//template <class T>
//class ThreadObject : public T {
//public:
//	ThreadMutex threadMutex;
//};
//
///* The same as ThreadObject but with T as a member instead of inherited */ //xxx: clean up naming
//template <class T>
//class SharedObject {
//public:
//	T item;
//	ThreadMutex threadMutex;
//	SharedObject() { item = T();};
//};

#endif /* THREAD_FUNCTION_H_ */
