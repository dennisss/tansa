#include <tansa/vehicle.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>



namespace tansa {


// THis should all move to the vehicle pool
int VehiclePool::connect(int lport, int rport, const char *laddr, const char *raddr) {

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
	memset((char *)&client_addr, 0, sizeof(client_addr));
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(rport);
	inet_pton(AF_INET, raddr != NULL? raddr : "127.0.0.1", &client_addr.sin_addr);

	running = true;
	if(pthread_create(&thread, NULL, vehicle_pool_thread, (void *) this) != 0) {
		running = false;
		return 1;
	}


	return 0;
}

int VehiclePool::disconnect() {
	running = false;
	close(netfd);

	/*
	for(auto &f : forwarders) { // TODO: Move some of this back to the vehicle class
		close(f.netfd);
	}
	forwarders.resize(0);
	*/

	pthread_join(thread, NULL);
	netfd = 0;
	thread = 0;
	return 0;
}

void VehiclePool::send_message(VehicleId id, mavlink_message_t *msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	//mavlink_finalize_message_chan(msg, mavlink_system.sysid, mavlink_system.compid, this->channel, msg->len);
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);


	const VehiclePoolConnection &c = connections[id];
	// TODO: Look up which client_addr to use based on the id
	sendto(netfd, buf, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
}

VehicleId VehiclePool::get_id(const struct sockaddr_in &addr) {
	//uint32_t a = ntohl(addr.sin_addr.s_addr);
	int port = ntohs(addr.sin_port);
	return (port - 14555) / 10;
}



/*
	We should map vehicles to
*/
void *vehicle_pool_thread(void *arg) {

	VehiclePool *pool = (VehiclePool *) arg;

	int res = 0;

	mavlink_message_t msg; memset(&msg, 0, sizeof(msg));
	mavlink_status_t status;


	char *buf = (char *) malloc(512);

	int nfds = 1; // + v->forwarders.size(); // TODO: Forwarders are hard as the vehicles in most cases are created afterwards
	struct pollfd *fds = (struct pollfd *) malloc(sizeof(struct pollfd) * nfds);

	// First descriptor for udp socket
	fds[0].fd = pool->netfd;
	fds[0].events = POLLIN; // TODO: Set netfd to non-blocking and allow the OS to buffer the sendto if needed

	/*
	// TODO: This assumes that the
	// Other ones for forwarders
	for(int i = 0; i < v->forwarders.size(); i++) {
		fds[1 + i].fd = v->forwarders[i].netfd;
		fds[1 + i].events = POLLIN;
	}
	*/


	std::map<VehicleId, VehiclePoolConnection> active_connections;
	pool->connectionsLock.lock();
	active_connections = pool->connections;
	pool->connectionsLock.unlock();

	printf("Waiting for messages...\n");

	// Poll for messages
	while(pool->running) {
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
			int nread = recvfrom(pool->netfd, buf, 512, 0, (struct sockaddr *)&addr, &addrlen);
			if(nread > 0) {

				// Perform filtering by network subnet
				// TODO

				// Find or create a connection
				VehicleId id = pool->get_id(addr);
				if(active_connections.count(id) == 0) {
					pool->connectionsLock.lock();
					if(pool->connections.count(id) == 0) {
						VehiclePoolConnection c;
						c.addr = addr;
						c.vehicle = Vehicle::Ptr(new Vehicle(pool, id));
						pool->connections[id] = c;
					}
					active_connections = pool->connections;
					pool->connectionsLock.unlock();
				}

				VehiclePoolConnection &c = active_connections[id];

				for(int i = 0; i < nread; i++) {
					if(mavlink_parse_char(c.vehicle->channel, buf[i], &msg, &status)) {
						c.vehicle->handle_message(&msg);
					}
				}

				// Also send to all forwarding channels
				for(auto &f : c.vehicle->forwarders) {
					sendto(f.netfd, buf, nread, 0, (struct sockaddr *) &f.client_addr, addrlen);
				}
			}
		}
		else { // Received data on one of the forwarder listeners
			/*
			// TODO: Refactor this

			struct sockaddr_in addr;
			socklen_t addrlen = sizeof(struct sockaddr_in);

			for(int i = 0; i < v->forwarders.size(); i++) {
				if(!(fds[1 + i].revents & POLLIN))
					continue;

				auto &f = v->forwarders[i];

				int nread = recvfrom(f.netfd, buf, 512, 0, (struct sockaddr *)&addr, &addrlen);

				// Register the client that is sending us messages
				f.client_addr.sin_port = addr.sin_port;
				f.client_addr.sin_addr = addr.sin_addr;

				// Send to vehicle
				sendto(v->netfd, buf, nread, 0, (struct sockaddr *)&v->client_addr, sizeof(v->client_addr));
			}
			*/

		}


		// Update all vehicles
		auto it = pool->connections.begin();
		while(it != pool->connections.end()) {
			VehiclePoolConnection &c = it->second;
			c.vehicle->cycle();

			// TODO: Vehicles that were just added may not be classified as connected yet
			// So: instead go by last packet received time
			// Cleanup by removing all vehicles that are not connected and not being used by someone else
			if(c.vehicle.use_count() == 1 && !c.vehicle->connected) {
				it = pool->connections.erase(it);
				continue;
			}

			it++;
		}
	}

	free(buf);
	free(fds);

	return NULL;

}





}
