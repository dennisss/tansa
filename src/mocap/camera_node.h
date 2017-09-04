#ifndef TANSA_MOCAP_CAMERA_NODE_H_
#define TANSA_MOCAP_CAMERA_NODE_H_

#include <tansa/time.h>
#include <tansa/core.h>
#include "camera_net.h"
#include "blob_detector.h"

#include <pthread.h>
#include <netinet/ip.h>


namespace tansa {

struct MocapCameraImage : Message {
	static const int ID = 1;

	Image *image;
};


/**
 * An abstraction over a device which produces image frame data
 */
class MocapCameraImagingInterface : public Channel {
public:

	virtual ~MocapCameraImagingInterface() {}


	virtual void start() = 0;
	virtual void stop() = 0;

	/**
	 * Should return the size of each expected frame
	 */
	virtual ImageSize getSize() = 0;

	// Should support starting with a new configuration
	// Should support getting out raw images
	// Should optionally also support getting out other formats such as jpeg and h264


protected:

};


/**
 * Runs onboard a motion capture camera to manage its state and act upon messages from a master
 */
class MocapCameraNode {
public:

	MocapCameraNode(Context *ctx, MocapCameraImagingInterface *interface, int model);

	int connect(const char *laddr = NULL, int lport = MOCAP_CAMERA_NODE_DEFAULT_PORT, int rport = MOCAP_CAMERA_MASTER_DEFAULT_PORT);
	void disconnect();



private:

	friend void *mocap_camera_node_thread(void *);


	void on_image(const MocapCameraImage *data);

	void send_blobs(const std::vector<ImageBlob> &blobs);
	void send_advert();
	void send_message(MocapCameraPacket *pkt);

	/**
	 * Enables/disables cpu throttling by the OS for improving latency when needed
	 * NOTE: Only available on linux
	 */
	void toggle_throttling(bool enabled);

	void display(Image *img, std::vector<ImageBlob> *blobs);

	void handle_message(MocapCameraPacket *msg);
	void cycle();

	int model;

	bool running = false;
	int netfd = 0;
	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;
	pthread_t thread = 0;


	bool active = false; /**< Whether or not we are  */
	Time lastKeepalive; /**< The last time the master node has told us to keep going */
	Time lastAdvertisement;
	Time lastFrame;

	MocapCameraImagingInterface *interface;

	BlobDetector *detector;

};

/**
 * Used to respond to messages coming from the master computer
 * @private
 */
void *mocap_camera_node_thread(void *);


}


#endif
