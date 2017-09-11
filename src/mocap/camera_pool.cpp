#include "camera_pool.h"

#include <poll.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>

using namespace std;


namespace tansa {





void MocapCameraPool::connect(int lport) {


	if((netfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket");
		return;
	}

	struct sockaddr_in addr;
	memset((char *)&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(lport);

	if (::bind(netfd, (struct sockaddr *) &addr, (socklen_t) sizeof(addr)) < 0) {
		perror("bind failed");
		close(netfd);
		netfd = 0;
		return;
	}

	int flags = fcntl(netfd, F_GETFL, 0);
	if(fcntl(netfd, F_SETFL, flags | O_NONBLOCK)) {
		perror("failed to make udp socket non-blocking");
		close(netfd);
		netfd = 0;
		return;
	}

	running = true;
	if(pthread_create(&thread, NULL, mocap_camera_pool_thread, (void *) this) != 0) {
		running = false;
		return;
	}


}



void MocapCameraPool::handleMessage(tansa::MocapCameraPacket *msg, const NetworkAddress &addr) {

	if(msg->type == CameraPacketAdvertise) {
		// If we get one from a non-registered camera, register the camera and send it a configuration

		MocapCameraPacketAdvertisement *advert = (MocapCameraPacketAdvertisement *) msg->data;

		// Ensure not already registered
		for(int i = 0; i < cameras.size(); i++) {
			if(cameras[i].address == addr)
				return;
		}

		MocapCamera c;
		c.id = lastCameraId++;
		c.model = advert->model;
		c.address = addr;
		c.image_width = advert->image_width;
		c.image_height = advert->image_height;
		c.lastReceived = Time::now();
		c.configured = false;
		cameras.push_back(c);

		cout << "Configure new camera" << endl;

		this->configure(c);
		this->publish_list();

		return;
	}

	// Otherwise, the packet will be for an already registered camera

	MocapCamera *c = NULL;
	for(int i = 0; i < cameras.size(); i++) {
		if(cameras[i].address == addr) {
			c = &cameras[i];
			break;
		}
	}

	// Unknown sender
	if(c == NULL) {
		cout << "CameraPool: Unknown sender" << endl;
		return;
	}

	c->configured = true;
	c->lastReceived = Time::now();

	// TODO: If we haven't received a real message in a while from the camera, then we should remove it from the list


	if(msg->type == CameraPacketBlobs) {
		// TODO: Publish with the originating camera id (the receiver will need to be aware that the ids are not array indexes)

		MocapCameraBlobsMsg *m = new MocapCameraBlobsMsg();

		m->cameraId = c->id;

		MocapCameraPacketBlobs *pb = (MocapCameraPacketBlobs *) msg->data;
		for(int i = 0; i < pb->nblobs; i++) {
			ImageBlob b;
			b.cx = pb->blobs[i].x;
			b.cy = pb->blobs[i].y;
			b.radius = pb->blobs[i].radius;
			m->blobs.push_back(b);
		}

		this->publish(m);
	}

}


void MocapCameraPool::cycle() {
	Time now = Time::now();


	// Look through all available cameras and send them a keep alive
	if(now.since(lastKeepaliveSent).seconds() > 0.5) {

		MocapCameraPacket pkt;
		pkt.type = CameraPacketKeepalive;
		pkt.size = 0;

		for(unsigned i = 0; i < cameras.size(); i++) {
			this->sendMessage(cameras[i], &pkt);
		}

		lastKeepaliveSent = now;
	}


	bool changed = false;

	// Cleanup dead cameras
	for(unsigned i = 0; i < cameras.size(); i++) {

		// Cameras totally timed out
		if(now.since(cameras[i].lastReceived).seconds() > 4) {
			cout << "CameraPool: #" << cameras[i].id << " timed out" << endl;
			cameras.erase(cameras.begin() + i);
			i--;
			changed = true;
		}
		// Resend configuration
		// TODO: Currently this will also do a single resend attempt
		else if(!cameras[i].configured &&  now.since(cameras[i].lastReceived).seconds() > 0.5) {
			configure(cameras[i]);
		}

	}


	if(changed) {
		publish_list();
	}

}

void MocapCameraPool::configure(const MocapCamera &cam) {

	unsigned len = sizeof(MocapCameraPacketConfig);
	MocapCameraPacket *pkt = (MocapCameraPacket *) malloc(sizeof(MocapCameraPacket) + len);
	pkt->type = CameraPacketConfig;
	pkt->size = len;

	MocapCameraPacketConfig *cfg = (MocapCameraPacketConfig *) pkt->data;
	cfg->mode = MocapCameraMode::Blob;

	// TODO: Populate configuration

	this->sendMessage(cam, pkt);
	free(pkt);
}



// TODO: This is very redundant with MocapCameraNode
void MocapCameraPool::sendMessage(const MocapCamera &cam, MocapCameraPacket *msg) {
	msg->magic[0] = 'T';
	msg->magic[1] = 'A';

	int len = sizeof(MocapCameraPacket) + msg->size;

	sendto(netfd, msg, len, 0, (const struct sockaddr *) cam.address.raw(), sizeof(struct sockaddr_in));
}


void *mocap_camera_pool_thread(void *arg) {

	MocapCameraPool *p = (MocapCameraPool *) arg;

	MocapCameraPacketParser parser;

	int res;

	int nfds = 1;
	struct pollfd fds[1];
	fds[0].fd = p->netfd;
	fds[0].events = POLLIN;

	char buf[1024];

	// Poll for messages
	while(p->running) {
		res = poll(fds, nfds, 100);
		if(res < 0) {
			// Error
		}
		else if(res == 0) {
			// timeout
		}
		else if(fds[0].revents & POLLIN) {
			struct sockaddr_in addr;
			socklen_t addrlen = sizeof(struct sockaddr_in);
			int nread = recvfrom(p->netfd, buf, 1024, 0, (struct sockaddr *)&addr, &addrlen);

			NetworkAddress remote_addr((struct sockaddr *) &addr);

			if(nread > 0) {
				for(int i = 0; i < nread; i++) {
					if(parser.parse(&buf[i])) {
						p->handleMessage(parser.get_packet(), remote_addr);
					}
				}
			}
		}

		p->cycle();
	}

	return NULL;





}



void MocapCameraPool::publish_list() {
	MocapCameraListMsg *msg = new MocapCameraListMsg();
	msg->cameras = cameras;
	this->publish(msg);
}


}
