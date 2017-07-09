#ifndef TANSA_CORE_MESSAGE_QUEUE_H_
#define TANSA_CORE_MESSAGE_QUEUE_H_

#include <deque>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace tansa {


/**
 * Thread safe interthread message queue
 *
 *
 * Each thread will have a single conditional variable which it will be waiting on. Whenever a message is sent to that thread, the central conditional_var will be notified which will trigger the thread to check through all subscriptions in the thread and
 */
template<class T>
class MessageQueue {
public:

	/**
	 * Initializes a new message queue
	 *
	 * @param maxSize the maximum number of items that can be in the queue at a single time
	 */
	MessageQueue(unsigned maxSize = 10) {
		this->maxSize = maxSize;
	}


	/**
	 * Adds an item to the back of the queue
	 *
	 * @param data the item to add
	 * @param blocking if true and the queue is full, it will block until it is empty, otherwise the oldest items will be removed in the case of overflow
	 */
	void push(const T &data, bool blocking = false) {
		// Must have the lock in order to put something
		std::unique_lock<std::mutex> lock(this->mutex);

		while(queue.size() > maxSize) {
			if(blocking) {
				cvar.wait(lock);
			}
			else {
				queue.pop_front();
			}
		}

		queue.push_back(data);

		lock.unlock();
		cvar.notify_one();
	}

	/**
	 * Retrieves one item from the beginning of the queue
	 *
	 *
	 */
	bool pop(T *out, bool blocking = true) {
		std::unique_lock<std::mutex> lock(this->mutex);

		if(queue.size() == 0) {
			if(blocking)
				cvar.wait(lock, [this]{ return this->queue.size() > 0; });
			else
				return false;
		}

		*out = queue.front();
		queue.pop_front();

		lock.unlock();
		cvar.notify_one();

		return true;
	}

	bool empty() {
		std::lock_guard<std::mutex> lock(this->mutex);
		return queue.size() == 0;
	}

private:

	std::deque<T> queue;
	unsigned maxSize;

	std::mutex mutex;
	std::condition_variable cvar;

};



}



#endif
