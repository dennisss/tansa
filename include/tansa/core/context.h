#ifndef TANSA_CORE_CONTEXT_H_
#define TANSA_CORE_CONTEXT_H_

#include "message_queue.h"
#include "subscription.h"

#include <map>
#include <vector>

namespace tansa {

class Channel;

/**
 * A single event pool and distributor. Typically there would be one of these per uncoupled thread, or a single one per group of worker threads
 */
class Context {
public:

	Context();

	/**
	 * Removes all subscriptions
	 */
	~Context();

	/**
	 * Cycles once through all
	 */
	void poll();

	/**
	 * If a thread is polling the context, calling this will wake it up
	 */
	void notify() { status.push(1, false); }

private:

	friend class Channel;

	//std::map<std::string, Subscription> external_subs; /**< All subscriptions to named inter-process or networked topics */

	// TODO: These also need to have message queues for storing received messages
	std::map<void *, std::vector<Subscription *>> subs; /**< All topics on which this node will receive messages */

	MessageQueue<unsigned> status;
};




}


#endif
