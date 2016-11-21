
#include <tansa/osc.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>

using namespace std;


int OSC::start(int lport) {
	if((netfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket");
		return 1;
	}

	memset((char *)&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(lport);

	if (::bind(netfd, (struct sockaddr *) &server_addr, (socklen_t) sizeof(server_addr)) < 0) {
		perror("bind failed");
		close(netfd);
		netfd = 0;
		return 1;
	}

	int flags = fcntl(netfd, F_GETFL, 0);
	if(fcntl(netfd, F_SETFL, flags | O_NONBLOCK)) {
		perror("failed to make udp socket non-blocking");
		close(netfd);
		netfd = 0;
		return 1;
	}


	// By default send to 127.0.0.1:rport
	/*
	memset((char *)&client_addr, 0, sizeof(client_addr));
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(rport);
	inet_pton(AF_INET, "127.0.0.1", &client_addr.sin_addr);
	*/

	running = true;
	if(pthread_create(&thread, NULL, osc_thread, (void *) this) != 0) {
		running = false;
		return 1;
	}

}

int OSC::stop() {


}

void OSC::process_message(char *buf) {

	OSCMessage msg;

	int i = 0;
	char *ptr;
	char *tok = strtok_r(buf, " \r\n\t", &ptr);

	while(tok != NULL) {
		if(i == 0) { // Got the address (split on '/')
			if(tok[0] != '/') { // Probably an invalid osc message
				return;
			}
			tok++; // Skip the first '/'


			char *atok = strtok(tok, "/");
			while(atok != NULL) {
				msg.address.push_back(string(atok));
				atok = strtok(NULL, "/");
			}

		}
		else {
			msg.args.push_back(string(tok));
		}

		tok = strtok_r(ptr, " \r\n\t", &ptr);
		i++;
	}

	if(listener != NULL) {
		listener(msg);
	}
}



void *osc_thread(void *arg) {

	OSC *o = (OSC *) arg;

	int res = 0;

	char *buf = (char *) malloc(512);

	struct pollfd fds[1];
	int nfds = 1;

	// First descriptor for udp socket
	fds[0].fd = o->netfd;
	fds[0].events = POLLIN;

	printf("Waiting for osc messages...\n");

	// Poll for messages
	while(o->running) {
		res = poll(fds, nfds, 500);
		if(res < 0) {
			// Error
		}
		else if(res == 0) {
			// timeout
		}
		else if(fds[0].revents & POLLIN) {
			struct sockaddr_in addr;
			socklen_t addrlen = sizeof(struct sockaddr_in);
			int nread = recvfrom(o->netfd, buf, 512, 0, (struct sockaddr *)&addr, &addrlen);

			// Register the client that is sending us messages
			//o->client_addr.sin_port = addr.sin_port;
			//o->client_addr.sin_addr = addr.sin_addr;

			if(nread > 0) {
				o->process_message(buf);
			}
		}


	}

	return NULL;

}
