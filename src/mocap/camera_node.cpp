#include "camera_node.h"

#include <unistd.h>
#include <arpa/inet.h>
#include <poll.h>
#include <fcntl.h>
#include <fstream>

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

using namespace std;


namespace tansa {


MocapCameraNode::MocapCameraNode(MocapCameraImagingInterface *interface) {
	this->interface = interface;
	interface->subscribe(&MocapCameraNode::on_image, this);

	ImageSize s = interface->getSize();

	detector = new BlobDetector(s.width, s.height);
	detector->set_threshold(200);
}

// TODO: Unsubscribe in destructor


int MocapCameraNode::connect(int lport, int rport) {

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
	// TODO: Perform broaddcasting
	memset((char *)&client_addr, 0, sizeof(client_addr));
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(rport);
	inet_pton(AF_INET, "127.0.0.1", &client_addr.sin_addr);

	running = true;
	if(pthread_create(&thread, NULL, mocap_camera_node_thread, (void *) this) != 0) {
		running = false;
		return 1;
	}

	// TODO: Instead this should wait for a network command
	interface->start();
}

void MocapCameraNode::disconnect() {


}


void MocapCameraNode::send_blobs(const vector<ImageBlob> &blobs) {

	int payloadSize = sizeof(MocapCameraPacketBlobs) + sizeof(MocapCameraPacketBlob)*blobs.size();

	MocapCameraPacket *pkt = (MocapCameraPacket *) malloc(sizeof(MocapCameraPacket) + payloadSize);

	pkt->type = CameraPacketBlobs;
	pkt->size = payloadSize;


	MocapCameraPacketBlobs *blobsPkt = (MocapCameraPacketBlobs *) pkt->data;
	blobsPkt->nblobs = blobs.size();

	for(int i = 0; i < blobs.size(); i++) {
		MocapCameraPacketBlob *b = &blobsPkt->blobs[i];
		b->x = blobs[i].cx;
		b->y = blobs[i].cy;
		b->radius = blobs[i].radius;
	}

	send_message(pkt);

	free(pkt);
}


void MocapCameraNode::send_advert() {


}

void MocapCameraNode::send_message(MocapCameraPacket *pkt) {
	pkt->magic[0] = 'T';
	pkt->magic[1] = 'A';

	int len = sizeof(MocapCameraPacket) + pkt->size;

	sendto(netfd, pkt, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
}

void MocapCameraNode::on_image(const MocapCameraImage *data) {

#ifdef USE_OPENCV
	Image original;
	data->image->copyTo(&original);
#endif

	vector<ImageBlob> blobs;


	int start_s=clock();

	detector->detect(data->image, &blobs);

	// the code you wish to time goes here
	int stop_s=clock();
	cout << "time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000 << endl;

	send_blobs(blobs);

#ifdef USE_OPENCV
	this->display(&original, &blobs);
	free(original.data);
#endif
}




// For visualizing the results
void MocapCameraNode::display(Image *imgRaw, std::vector<ImageBlob> *blobs) {

#ifdef USE_OPENCV
	cv::Mat gray(imgRaw->height, imgRaw->width, CV_8UC1, imgRaw->data);

	cv::Mat img;
	cv::cvtColor(gray, img, CV_GRAY2BGR);


	for(size_t i = 0; i < blobs->size(); i++) {
		//cout << blobs[i].cx << " " << blobs[i].cy << ": " << blobs[i].area << endl;

		cv::Point center((*blobs)[i].cx, (*blobs)[i].cy);
		int radius = (*blobs)[i].radius;
		// circle center
		cv::circle(img, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
		// circle outline
		cv::circle(img, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
	}

	cv::Mat imageSmall;
	cv::resize(img, imageSmall, cv::Size(img.cols * 0.7, img.rows * 0.7), 0, 0, CV_INTER_LINEAR);

	cv::namedWindow("Blobs", cv::WINDOW_AUTOSIZE);
	cv::imshow("Blobs", imageSmall);
	cv::waitKey(1);
#endif

}


void MocapCameraNode::handle_message(MocapCameraPacket *msg) {

	if(msg->type == CameraPacketConfig) {
		if(active) {
			interface->stop();
		}

		// TODO: Use the provided config
		interface->start();
		active = true;
	}
	else if(msg->type == CameraPacketKeepalive) {
		lastKeepalive = Time::now();
	}
}

void MocapCameraNode::cycle() {

	Time now = Time::now();

	if(active) {
		if(now.since(lastKeepalive).seconds() >= 2) {
			// TODO: Also reset to broadcast address for client
			// Turn ourselves off
			interface->stop();
			active = false;
		}
	}
	else {
		if(now.since(lastAdvertisement).seconds() >= 0.5) {
			// Broadcast an advertisemnet
			send_advert();
		}
	}


}


void MocapCameraNode::toggle_throttling(bool enabled) {
	ofstream f("/etc/default/cpufrequtils", ofstream::trunc);
	if(!f.is_open()) {
		cout << "Throttling failed!" << endl;
		return;
	}
	f << "GOVERNER=\"" << (enabled? "ondemand" : "performance") << "\"" << endl;
	f.close();
	system("/etc/init.d/cpufrequtils restart"); // TODO: Check the return value
}


void *mocap_camera_node_thread(void *arg) {

	MocapCameraNode *n = (MocapCameraNode *) arg;

	MocapCameraPacketParser parser;

	int res;

	int nfds = 1;
	struct pollfd fds[1];
	fds[0].fd = n->netfd;
	fds[0].events = POLLIN;

	char buf[512];

	// Poll for messages
	while(n->running) {
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
			int nread = recvfrom(n->netfd, buf, 512, 0, (struct sockaddr *)&addr, &addrlen);

			// Register the client that is sending us messages
			n->client_addr.sin_port = addr.sin_port;
			n->client_addr.sin_addr = addr.sin_addr;

			if(nread > 0) {
				for(int i = 0; i < nread; i++) {
					// TODO: Pick comm channel based on the ip/port from which the data was received

					if(parser.parse(&buf[i])) {
						n->handle_message(parser.get_packet());
					}
				}
			}
		}

		n->cycle();
	}

	return NULL;


}







}
