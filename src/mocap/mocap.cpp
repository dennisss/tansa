#include <tansa/mocap.h>
#include <tansa/vehicle.h>
#include <tansa/time.h>

#include <iostream>

#include "optitrack/natnet_client.h"
#include "rigid_body_tracker.h"

namespace tansa {


Mocap::Mocap(const MocapOptions &options) {
	client = nullptr;
	this->options = options;

	if(options.mode == MocapRigidBodyFromCloud /* || options.useActiveBeacon */)
		tracker = new RigidBodyTracker();
	else
		tracker = NULL;
}

Mocap::~Mocap() {
	if(client != nullptr) {
		delete client;
	}
}


int Mocap::connect(string iface_addr, string server_addr) {
	client = new optitrack::NatNetClient();

	if(client->connect((char *)iface_addr.c_str(), server_addr.c_str(), optitrack::NatNetUnicast) != 0){
		delete client;
		return 1;
	}

	client->subscribe(&Mocap::onNatNetFrame, this);

	return 0;
}

int Mocap::disconnect() {
	client->disconnect();
	return 0;
}

void Mocap::track(Vehicle *v, int id, const vector<Vector3d> &markers) {
	this->tracked[id] = v;

	// TODO: The tracker ones are never reset
	// We should clear them
	if(this->tracker != NULL && options.mode == MocapRigidBodyFromCloud) {
		this->tracker->track(v);
	}
}

inline Vector3d opti_pos_to_enu(double x, double y, double z) {

}

inline Vector3d opti_orient_to_enu(double x, double y, double z) {

}




void Mocap::onNatNetFrame(const optitrack::NatNetFrame *frame) {

	// Time at which the frame was acquired
	tansa::Time t = Time::now().subtract( Time((frame->latency / 1000.0) + client->get_connection_latency()) );


	for(int i = 0; i < frame->rigidBodies.size(); i++){

		const optitrack::NatNetRigidBody *rb = &frame->rigidBodies[i];

	//	cout << rb->markerPositions.size() << endl;

		int id = rb->id;

		// TODO: If there is an unidentified body, use active IR beacon to find correspondences (do this in another thread?)
		// Only continue if a key exists for this rigid body
		if(this->tracked.count(id) == 0){
			continue;
		}


		// Don't send it if it wasn't tracked correctly
		// TODO: We may want to trigger some failsafe in this case (also, after some time of tracking lose, the id->vehicle mapping should be discarded as the tracking may have picked up a different drone by that point)

		if(!rb->isTrackingValid())
			continue;


		// Conversion from default Optitrack coordinate system to ENU
		// Invert x and swap y & z (for version 1.7+)
		Vector3d pos(
			-rb->x,
			rb->z,
			rb->y
		);

		Quaterniond quat(
			rb->qw,
			-rb->qx,
			rb->qz,
			rb->qy
		);

		this->tracked[id]->mocap_update(pos, quat, t);

	}



	// Maintain the beacon floating high for most of the time by constantly pinging it
	if(t.since(lastBeaconPing).seconds() > 3) {
		for(auto entry : this->tracked) {
			entry.second->set_beacon(true);
		}

		lastBeaconPing = t;
	}




	// Grab other markers that were triangulated but not attached to rigid bodies
	// TODO: If we want to exclude rigid bodies, we should switch this to use otherMarkers
	vector<Vector3d> markers;
	for(int i = 0; i < frame->labeledMarkers.size(); i++) {

		if(frame->labeledMarkers[i].isOccluded()) {
			continue;
		}

		markers.push_back(Vector3d(
			-frame->labeledMarkers[i].x,
			frame->labeledMarkers[i].z,
			frame->labeledMarkers[i].y
		));
	}


	if(tracker != NULL) {
		tracker->update(markers, t);
	}
	
}

void Mocap::resync() {


}


void Mocap::start_recording() {
	client->send_message("StartRecording");
}
void Mocap::stop_recording() {
	client->send_message("StopRecording");
}

}
