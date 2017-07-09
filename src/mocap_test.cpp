
#include <tansa/core.h>
#include "mocap/camera_pool.h"
#include "mocap/reconstruction.h"

#ifdef  __linux__
#include <sys/signal.h>
#endif

#include <unistd.h>
#include <iostream>

using namespace std;
using namespace tansa;


static bool running;

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}

/*
void send_image() {

	json jsonStatus;

	jsonStatus["type"] = "camera_packet";

	jsonStatus["image"] = base64_encode(&my_buffer[0], my_buffer.size());

	tansa::send_message(jsonStatus);
}
*/

void on_camera_list(const MocapCameraListMsg *msg, void *arg) {

	json list;
	list["type"] = "camera_list";

	json cameras = json::array();
	for(int i = 0; i < msg->cameras.size(); i++) {
		json c;
		c["width"] = msg->cameras[i].image_width;
		c["height"] = msg->cameras[i].image_height;
		c["id"] = msg->cameras[i].id;
		cameras.push_back(c);
	}

	list["cameras"] = cameras;
	tansa::send_message(list);
}

void on_camera_blobs(const MocapCameraBlobsMsg *msg, void *arg) {

	json jsonStatus;

	jsonStatus["type"] = "camera_packet";

	jsonStatus["id"] = msg->cameraId;

	json blobs = json::array();
	for(int i = 0; i < msg->blobs.size(); i++) {
		json b;
		b["x"] = msg->blobs[i].cx;
		b["y"] = msg->blobs[i].cy;
		b["r"] = msg->blobs[i].radius;
		blobs.push_back(b);
	}

	jsonStatus["blobs"] = blobs;

	tansa::send_message(jsonStatus);
}


int main(int argc, char *argv[]) {
	tansa::init(true);
	tansa::Context ctx;


	MocapCameraPool p;

	p.subscribe(&ctx, on_camera_list);
	p.subscribe(&ctx, on_camera_blobs);

	p.connect();

	running = true;
	while(running) {
		ctx.poll();
	}


	printf("Done!\n");

}
