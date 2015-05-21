/*
 * Thread.h
 *
 *  Created on: Mar 7, 2014
 *      Author: tpetri
 */

#ifndef THREAD_H_
#define THREAD_H_

namespace ait
{

class Thread
{
public:
	Thread() : mbRunning(false), mbStopping(false), handle(0) {}
	virtual ~Thread();

	//call the run() method in a separate thread.
	void Start();
	//notify the thread that it should think about stopping
	void Stop() { mbStopping = true; }
	//wait for thread to stop.
	void Join();

	//returns true if the thread was notified that it should stop
	const bool &IsStopping() const { return mbStopping; }
	//returns true if the thread was started
	bool IsRunning() { return mbRunning; }

	//this is the main method of the thread, override this.
	virtual void Run() = 0;

protected:
	static void* threadproc(void* param);
	static void threadproc2(void *param);

protected:
	bool mbRunning;
	bool mbStopping;
	static bool mbInitialized;
	void *handle;			//this will be allocated in the respective implementations for win32 and posix
};

}

#endif /* THREAD_H_ */
