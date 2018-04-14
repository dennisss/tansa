#include "crazyradio.h"

namespace tansa {
namespace cf {

/**
 * Manages all of the radios connected to the computer
 */
class ConnectionPool {

public:
	// Takes some number of RadioUri's and 

	void addConnection(const RadioUri &uri, int handler);
	void removeConnection(const RadioUri &uri);

	void queueMessage(const RadioUri &uri, const crtp_message_t &msg);

private:

	// Need a list of all radio uris that we have added with handlers

	// Need a bunch of queues for outgoing messages



}


}
}