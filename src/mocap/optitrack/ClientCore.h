#ifndef NATNET_CLIENTCORE_H_
#define NATNET_CLIENTCORE_H_

#include "NatNetTypes.h"

#include <tansa/time.h>

#include <pthread.h>

#include <vector>
#include <string>

// Tracks memory allocations so that garbage collection is easier
class Memory {
public:
	Memory();
	~Memory();
	void *alloc(size_t size); // Works like malloc
private:
	std::vector<void *> memory;
};




// Data callback function prototype
typedef void ( *funcDataCallback) (sFrameOfMocapData* pFrameOfData, void* pUserData);


class ClientCore {

	friend void *data_server(void *arg);
	friend void *cmd_server(void *arg);

public:

	ClientCore(int type);
	~ClientCore();


	/*
		Setup all the sockets and start the data servers
		clientAddr should be the address on the same interface as the server
	*/
	void start(const char *clientAddr, const char *serverAddr, int cmdPort, int dataPort);
	void stop();


	/* Sends a ping command to the server */
	void ping();



	funcDataCallback data_callback;
	void *data_callback_arg;
	// also include the data associated with the callback





	void sendPacket(sPacket *packet);


	void setMulticastAddress(std::string multicast_addr) { this->multicast_addr = multicast_addr; };


//	struct sockaddr_in multi_addr;
//	struct sockaddr_in cmd_addr;


	// Data buffers
	sFrameOfMocapData *frame;
	sDataDescriptions *dataDescs;

private:
	int type;

	bool running;

	int data_socket;
	pthread_t data_thread;
	int cmd_socket;
	pthread_t cmd_thread;

	// ip/port on this computer to
	//struct sockaddr_in client_addr;
	int cmd_port;
	int data_port;
	std::string client_addr;
	std::string server_addr;

	std::string multicast_addr;

	tansa::Time lastping;

	void unpack(char *pData);
	void unpackFrame(char *ptr);
	void unpackDataDescs(char *ptr);

	Memory *frameMem;

};


// Private functions for reading in separate threads
void *data_server(void *arg);
void *cmd_server(void *arg);


#endif
