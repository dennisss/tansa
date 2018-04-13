#ifndef TANSA_MOCAP_CAMERA_POOL_H_
#define TANSA_MOCAP_CAMERA_POOL_H_

#include <tansa/core.h>
#include <tansa/net.h>
#include <tansa/time.h>
#include "camera_net.h"

#include "blob_detector.h" // For ImageBlob type

#include <pthread.h>
#include <vector>


namespace tansa {



struct MocapCamera {
	// Need the model information
	// Need the current configuration
	// Need some unique id for it
	// Need the remote address

	unsigned id;

	unsigned model;

	NetworkAddress address;

	unsigned image_height;
	unsigned image_width;

	Vector3d position;
	Quaterniond orientation;

	Time lastReceived; /**< Last  */
	bool configured = false;
};



// Published when the list has changed
struct MocapCameraListMsg : Message {
	static const unsigned ID = 1;

	std::vector<MocapCamera> cameras;
};

struct MocapCameraBlobsMsg : Message {
	static const unsigned ID = 2;

	unsigned cameraId;
	std::vector<ImageBlob> blobs;

};



/**
 * For communicating with and configuring many networked camera nodes
 */
class MocapCameraPool : public Channel {
public:

	/**
	 * Change the configured settings used by all the cameras
	 */
	void setSettings();


	void connect(int lport = MOCAP_CAMERA_MASTER_DEFAULT_PORT);
	void disconnect();

	// Should be able to get the list of cameras

	// Should be able configure the individual cameras

	// Should be able to listen for raw blob/jpeg data

private:

	friend void *mocap_camera_pool_thread(void *);

	void handleMessage(MocapCameraPacket *msg, const NetworkAddress &addr);
	void cycle();

	void configure(const MocapCamera &cam);

	void sendMessage(const MocapCamera &cam, MocapCameraPacket *msg);

	void publish_list();

	// maybe  use a map
	std::vector<MocapCamera> cameras; /**< All cameras that are currently connected */
	unsigned lastCameraId = 0;


	int netfd;
	bool running;
	pthread_t thread;

	Time lastKeepaliveSent;

};

void *mocap_camera_pool_thread(void *);


}

#endif
