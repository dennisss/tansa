#ifndef TANSA_OSC_H_
#define TANSA_OSC_H_

#include <pthread.h>
#include <netinet/ip.h>

#include <vector>
#include <string>

struct OSCMessage {
	std::vector<std::string> address;
	std::vector<std::string> args;
};

typedef void (*osc_listener)(OSCMessage &msg);


class OSC {

public:

	/**
	 * Starts an OSC Server listening for messages on the given port
	 */
	int start(int lport);

	int stop();

	void set_listener(osc_listener l) { this->listener = l; }

private:

	friend void *osc_thread(void *arg);

	void process_message(char *buf);


	bool running = false;

	int netfd = 0;

	pthread_t thread = 0;

	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;

	osc_listener listener = NULL;

};

void *osc_thread(void *arg);


#endif
