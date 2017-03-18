#ifndef TANSA_NATNET_CLIENT_H_
#define TANSA_NATNET_CLIENT_H_

#include "natnet.h"

#include <tansa/time.h>
#include <tansa/channel.h>

#include <pthread.h>
#include <netinet/ip.h>

#include <vector>
#include <string>


namespace tansa {
namespace optitrack {

/**
 * For communicating with applications such as OptiTrack Motive via the NatNet protocol
 */
class NatNetClient : public Channel {
public:

	NatNetClient();
	~NatNetClient();

	/**
	 * Starts listening for connections
	 *
	 * @param client_addr the local interface address
	 * @param server_addr the ip address of the remote application computer. when in multicast mode, this will be used as the multicast address (a default is available if this is null). if in unicast mode and not specified, this will default to the broadcast address of the interface until the server is found
	 * @param type
	 * @param cmd_port
	 * @param data_port
	 * @return 0 if successfully started (this does not necessarily mean that we are connected)
	 */
	int connect(const char *client_addr, const char *server_addr, NatNetConnectionType type = NatNetMulticast, int cmd_port = DEFAULT_COMMAND_PORT, int data_port = DEFAULT_DATA_PORT);

	void disconnect();


	/**
	 * Sends a ping message to the server
	 */
	void ping();

	void send_message(const char *msg);

	void send_message_waiting(const char *msg);

	double get_connection_latency() { return connection_latency.seconds(); }


	friend void *natnet_data_server(void *arg);
	friend void *natnet_cmd_server(void *arg);

private:

	void send_packet(const NatNetPacket &packet);

	void handle_packet(const NatNetPacket *pkt);


	NatNetConnectionType type;
	bool running;

	int data_socket;
	pthread_t data_thread;
	int cmd_socket;
	pthread_t cmd_thread;

	// ip/port on this computer to
	//struct sockaddr_in client_addr;
	int cmd_port;
	int data_port;

	struct sockaddr_in client_addr;
	struct sockaddr_in server_addr;

	tansa::Time lastping;

	tansa::Time connection_latency;


	/**
	 * Version of the protocol used by the server
	 */
	NatNetVersion natNetVersion = {0, 0, 0, 0};

	/**
	 * Version of the server application
	 */
	NatNetVersion serverVersion = {0, 0, 0, 0};

};


// Private functions for reading in separate threads
void *natnet_data_server(void *arg);
void *natnet_cmd_server(void *arg);


}
}

#endif
