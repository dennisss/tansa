/*
	This file is the primary file that performs motion capture
*/

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

static bool calibrationMode = false;
static bool calibrationSampleNext = false;
static vector<MocapCameraBlobsMsg> calibrationSamples;



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

	if(calibrationSampleNext) {
		calibrationSamples.push_back(*msg);
		cout << "Took calibration sample " << calibrationSamples.size() << endl;
		calibrationSampleNext = false;
	}


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



void socket_on_message(const json &data) {

	string type = data["type"];


	if(type == "calibration_start") {
		// TODO: Block calibration state if flying or prepared to fly (with drones connected)
		cout << "Start calibration" << endl;
		calibrationSamples.resize(0);
		calibrationMode = true;
	}
	else if(type == "calibration_cancel") {
		cout << "Exit calibration" << endl;
		calibrationMode = false;
	}
	else if(type == "calibration_sample") { // Takes an image on the next frame
		calibrationSampleNext = true;
	}
	else if(type == "calibration_finish") {
		cout << "Computing calibration" << endl;
		
		calibrationMode = false;
	}
	else {
		// TODO: Send an error message back to the browser
		printf("Unexpected message type recieved!\n");
	}
}


int main(int argc, char *argv[]) {
	tansa::init(true);
	tansa::on_message(socket_on_message);

	
	tansa::Context ctx;


	MocapCameraPool pool;

	// TODO: Doesn't currently work for multiple subscribers
	pool.subscribe(&ctx, on_camera_list);
	pool.subscribe(&ctx, on_camera_blobs);

	PointReconstructor recon(&ctx, &pool);

	pool.connect();

	running = true;
	Rate r(30);
	while(running) {
		ctx.poll();
		r.sleep(); // TODO: Only required right now as poll is setup like pollOnce right now
	}


	printf("Done!\n");

}
