#include <tansa/mocap.h>
#include <tansa/vehicle.h>
#include <tansa/time.h>

#include <iostream>

#include "optitrack/NatNetClient.h"
#include "optitrack/NatNetTypes.h"

namespace tansa {

static Vector3d lastPosition(0,0,0);
static bool found = false;
static bool beaconOn = false;
static Quaterniond startOrient;
static int frameI = 0;

static Vector3d velocity(0,0,0);

Mocap::Mocap() {


}


int Mocap::connect(string iface_addr, string server_addr) {
	client = new NatNetClient(ConnectionType_Unicast);

	if(client->Initialize((char *)iface_addr.c_str(), server_addr.c_str()) != 0){
		delete client;
		return 1;
	}

	client->SetDataCallback(mocap_callback, (void *) this);

	return 0;
}

int Mocap::disconnect() {

	return 0;
}

void Mocap::track(Vehicle *v, int id) {
	this->tracked[id] = v;
}

void mocap_callback(sFrameOfMocapData* pFrameOfData, void* pUserData){

	Mocap *inst = (Mocap *) pUserData;

	tansa::Time t = Time::now(); // TODO: Replace with the mocap timestamp

	for(int i = 0; i < pFrameOfData->nRigidBodies; i++){

		sRigidBodyData *rb = &pFrameOfData->RigidBodies[i];

		int id = rb->ID;

		// TODO: If there is an unidentified body, use active IR beacon to find correspondences (do this in another thread?)
		// Only continue if a key exists for this rigid body
		if(inst->tracked.count(id) == 0){
			continue;
		}


		// Don't send it if it wasn't tracked correctly
		// TODO: We may want to trigger some failsafe in this case (also, after some time of tracking lose, the id->vehicle mapping should be discarded as the tracking may have picked up a different drone by that point)
		bool bTrackingValid = rb->params & 0x01;
		if(!bTrackingValid)
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

		inst->tracked[id]->mocap_update(pos, quat, t);

//		cout << pos.transpose() << endl;

	}



	// Grab other markers that were triangulated but not attached to rigid bodies
	vector<Vector3d> markers;
	markers.resize(pFrameOfData->nOtherMarkers);
	for(int i = 0; i < markers.size(); i++) {
		markers[i] = Vector3d(
			-pFrameOfData->OtherMarkers[i][0],
			pFrameOfData->OtherMarkers[i][2],
			pFrameOfData->OtherMarkers[i][1]
		);

		//cout << markers[i].transpose() << endl;

	}

	// Markers is now the full

	/*

	Vehicle *v = inst->tracked[1];


	Vector3d mainMarker;
	int mainMarkerI = -1;
	double minError = 9999.0;

	// Find primary marker
	for(int i = 0; i < markers.size(); i++) {

		Vector3d m = markers[i];

		// Initial scan
		if(!found) {
			if(m.norm() < 0.5) { // Look for something close to the origin
				lastPosition = m;
				startOrient = v->onboardState.orientation;
				found = true;

				cout << "Found marker at: " << m.transpose() << endl;
				break;
			}
		}
		// Track
		else {
			double e = (m - lastPosition).norm();
			if( e < minError) { // e < 0.035 &&
				minError = e;
				mainMarkerI = i;
				mainMarker = m;

				lastPosition = mainMarker;
			}
		}
	}



	// Look for alignment beacon
	if(beaconOn) {
		for(int i = 0; i < markers.size(); i++) {
			if(mainMarkerI == i) {
				continue;
			}




		}

	}



	frameI++;

	if(found) {
		Vector3d pos = startOrient * lastPosition;
		pos.z() = lastPosition.z();
//		v->mocap_update(pos, Quaterniond(1,0,0,0), t);


	}
	*/

	/*
		If we have 2 markers, then we can determine auto-gyro bias estimation

		- We record initial yaw orientation y0 with mocap orientation ym0 at time 0
		- Later we record any set: yt and ymt at time 't'
		- If the bias was perfect, then yt - y0 == ymt - ym0
			- (yt - y0) - (ymt - ym0) = ydiff is the unexplained rotation angle integrated from the gyro
			- differentiating, we get a bias of (ydiff / t) = b
			- we will use gradient descent to incrementally add to the current gyro bias until
	*/

	/*
	// Toggle the beacon in short bursts
	if(found && frameI % 120 == 0) {
		v->set_beacon(true);
		beaconOn = true;
	}
	else if((frameI - 30) % 120 == 0 && beaconOn) { // Turn off in 30 frames
		v->set_beacon(false);
		beaconOn = false;
	}
	*/
}

}
