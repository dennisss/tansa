#include "natnet_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>

#include <arpa/inet.h>


#include <iostream>
using namespace std;


namespace tansa {
namespace optitrack {


NatNetClient::NatNetClient() : lastping(0,0) {
	running = false;
}


NatNetClient::~NatNetClient(){
	if(running) {
		disconnect();
	}
}


// TODO: Abstract this across the whole platform
int create_socket(){
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	#ifdef SO_REUSEPORT
	int reuse = 1;
	if(setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(int)) == -1){
		printf("Failed to make socket reusable\n");
		exit(1);
		return -1;
	}
	#endif

	return sock;
}

// TODO: Return errors from here
int NatNetClient::connect(const char *clientAddr, const char *serverAddr, NatNetConnectionType type, int cmdPort, int dataPort) {

	this->type = type;
	this->cmd_port = cmdPort;
	this->data_port = dataPort;
	this->server_addr = serverAddr;
	this->client_addr = clientAddr;

	int optval;

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));


	this->cmd_socket = create_socket();

	if(type == NatNetMulticast || server_addr.length() == 0) {
		// Enable broadcasting
		optval = 1;
		if(setsockopt(this->cmd_socket, SOL_SOCKET, SO_BROADCAST, (void *)&optval, sizeof(optval)) == -1) {
			perror("setsockopt (SO_BROADCAST)");
			exit(1);
		}
	}


	// Bind to command address
	sa.sin_port = htons(cmd_port);
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY); // TODO: This should be the local interface ip
	if(::bind(this->cmd_socket, (const struct sockaddr*)&sa, sizeof(sa)) < 0){
		printf("NatNetClient: failed to bind command socket\n");
	}





	this->data_socket = create_socket();

	// Bind to data address
	sa.sin_port = htons(data_port);
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY); // TODO: This should be the interface ip
	if(::bind(this->data_socket, (const struct sockaddr*)&sa, sizeof(sa)) < 0){
		printf("NatNetClient: failed to bind data socket\n");
	}

	if(type == NatNetMulticast) {
		// Join multicast group
		ip_mreq mreq;

		inet_pton(AF_INET, multicast_addr.c_str(),  &mreq.imr_multiaddr); //&this->multi_addr.sin_addr);
		//mreq.imr_multiaddr = this->multi_addr.sin_addr;

		int r;

		inet_pton(AF_INET, clientAddr, &mreq.imr_interface.s_addr);
		//mreq.imr_interface.s_addr = htonl(INADDR_ANY);
		r = setsockopt(this->data_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *)&mreq, sizeof(mreq)); // TODO: Error check this

		if(r < 0){
			printf("Failed to join multicast group!\n");
		}
	}


	// Make the buffer bigger
	optval = 0x100000;
	setsockopt(this->data_socket, SOL_SOCKET, SO_RCVBUF, (void *)&optval, sizeof(optval)); // TODO: Error check this




	running = true;


	// Start data server
	if(pthread_create(&this->data_thread, NULL, natnet_data_server, this) != 0){
		printf("NatNetClient: failed to create data thread\n");
	}

	// Start command server
	if(pthread_create(&this->cmd_thread, NULL, natnet_cmd_server, this) != 0){
		printf("NatNetClient: failed to create command thread\n");
	}

	if(type == NatNetUnicast) {
		// Initial ping to connect to server
		this->ping();
		lastping = tansa::Time::now();
	}


	return 0;

}


void NatNetClient::disconnect(){
	this->running = false;

	// Close the sockets
	close(this->data_socket);
	close(this->cmd_socket);


	// Join the threads
	pthread_join(this->data_thread, NULL);
	pthread_join(this->cmd_thread, NULL);
}


void NatNetClient::send_packet(const NatNetPacket &packet) {

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));

	sa.sin_family = AF_INET;
	sa.sin_port = htons(cmd_port);


	if(server_addr.length() == 0) { // If server address not specified, broadcast
		//sa.sin_addr.s_addr = INADDR_BROADCAST;
		inet_pton(AF_INET, "192.168.1.255", &sa.sin_addr);
	}
	else {
		printf("Regular send %s\n", server_addr.c_str());
		inet_pton(AF_INET, server_addr.c_str(), &sa.sin_addr);
	}

	int res = sendto(this->cmd_socket, &packet, 4 + packet.size, 0, (struct sockaddr *)&sa, sizeof(struct sockaddr_in));


	if(res == -1) {
		printf("Sending error\n");
		// Error
	}

}



void NatNetClient::ping(){
	NatNetPacket pkt;
	pkt.type = NatNetPacket::Ping;
	pkt.size = 0;
	this->send_packet(pkt);
}

void NatNetClient::send_message(const char *msg){
	NatNetPacket pkt;
	pkt.type = NatNetPacket::Request;
	pkt.size = strlen(msg) + 1;
	strcpy(pkt.payload, msg);
	this->send_packet(pkt);
}



void NatNetClient::handle_packet(const NatNetPacket *pkt) {

	if(pkt->type == NatNetPacket::Frame) {
		NatNetFrame f;
		NatNetFrame::Parse(pkt->payload, natNetVersion, &f);
		this->publish(f);
	}
	else if(pkt->type == NatNetPacket::ModelDef) {
		NatNetDescriptions d;
		NatNetDescriptions::Parse(pkt->payload, natNetVersion, &d);
		this->publish(d);
	}
	else{
		// unrecognized
	}

}



void *natnet_data_server(void *arg){
	NatNetClient *c = (NatNetClient *) arg;

	char buffer[20000];
	struct sockaddr_in sa;
	socklen_t addr_len = sizeof(struct sockaddr_in);


	struct pollfd fds[1];
	int nfds = 1;

	// First descriptor for udp socket
	fds[0].fd = c->data_socket;
	fds[0].events = POLLIN;

	int res;

	while(c->running) {

		res = poll(fds, nfds, 50);
		if(res < 0) {
			// Error
		}
		else if(res == 0) {
			// timeout
		}
		else if(fds[0].revents & POLLIN) {
			int r = recvfrom(c->data_socket, buffer, sizeof(buffer), 0, (sockaddr *) &sa, &addr_len);

			if(r == 0){
				continue;
			}
			else if(r < 0){
				continue;
			}

			c->handle_packet((NatNetPacket *) buffer);
		}


		// Every half second, send a ping request
		if(c->type == NatNetUnicast) {
			tansa::Time t = tansa::Time::now();
			if(t.since(c->lastping).seconds() >= 0.5) {
				c->ping();
				c->lastping = t;
			}
		}

	}

	return NULL;
}

void *natnet_cmd_server(void *arg) {
	NatNetClient *c = (NatNetClient *) arg;

	int r;
	struct sockaddr_in sa;
	socklen_t addr_len = sizeof(struct sockaddr);

	NatNetPacket pkt;

    while(c->running) {
        // blocking
        r = recvfrom(c->cmd_socket, (char *) &pkt, sizeof(pkt), 0, (struct sockaddr *)&sa, &addr_len);

        if((r == 0) || (r < 0))
            continue;


		// TODO: Block messages received from ourselves (via broadcast)

        // handle command
		switch(pkt.type) {
		case NatNetPacket::ModelDef:
		case NatNetPacket::Frame:
			c->handle_packet(&pkt);
			break;
		case NatNetPacket::PingResponse: {
			NatNetSender *s = (NatNetSender *) pkt.payload;

			// Register the address of the server if none was originally specified
			if(c->server_addr.length() == 0) {
				char buf[32];
				inet_ntop(AF_INET, &sa.sin_addr, buf, sizeof(buf));
				c->server_addr = buf;
			}

			for(int i = 0; i < 4; i++) {
				c->natNetVersion[i] = s->protocolVersion[i];
				c->serverVersion[i] = s->appVersion[i];
			}

			break;
		}
		case NatNetPacket::Response:
			printf("NatNet got respone\n");
			// payload is either a 4byte integer or a string
			break;
		case NatNetPacket::UnrecognizedRequest:
			printf("NatNet received 'unrecognized request'\n");
			break;
		case NatNetPacket::MessageString:
			printf("NatNet received message: %s\n", pkt.payload);
			break;
		case NatNetPacket::Ping:
			printf("Pinged...\n");
			break;
		default:
			printf("Got %d\n", pkt.type);
		}
	}

	return NULL;
}


}
}
