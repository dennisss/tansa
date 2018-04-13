/*
	This is a CLI utility for calibrating the motion capture system
	This will generate the files needed to run the manager

	TODO: This also needs to show the images as they will be needed in order to help position the wand
	- SO: this needs to trasmit them to the web gui
*/

#include <tansa/core.h>
#include "mocap/camera_pool.h"
#include "mocap/calibration.h"


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

	MocapCameraPool pool;

	pool.subscribe(&ctx, on_camera_list);
	pool.subscribe(&ctx, on_camera_blobs);

	pool.connect();

	running = true;
	Rate r(30);
	while(running) {
		ctx.poll();
		r.sleep(); // TODO: Only required right now as poll is setup like pollOnce right now
	}


	printf("Done!\n");

}
