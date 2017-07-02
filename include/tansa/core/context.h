#ifndef TANSA_CORE_CONTEXT_H_
#define TANSA_CORE_CONTEXT_H_

#include "message_queue.h"

#include <map>

namespace tansa {

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
	 * Loops waiting for
	 */
	void poll();

private:

	std::map<std::string, Subscription> external_subs; /**< All subscriptions to named inter-process or networked topics */

	// TODO: These also need to have message queues for storing received messages
	std::map<Channel *, Subscription> subs; /**< All topics to which from which this node will receive messages from */

	MessageQueue<unsigned> status;
};




}


#endif
