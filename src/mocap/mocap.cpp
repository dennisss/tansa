#include <tansa/mocap.h>
#include <tansa/vehicle.h>
#include <tansa/time.h>

#include "optitrack/NatNetClient.h"
#include "optitrack/NatNetTypes.h"

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

	Time t = Time::now(); // TODO: Replace with the mocap timestamp

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


	}


/*
	// Grab other markers that were triangulated but not
	vector<Vector3d> markers;
	markers.resize(pFrameOfData->nOtherMarkers);
	for(int i = 0; i < markers.size(); i++) {
		markers[i] = Vector3d(
			-pFrameOfData->OtherMarkers[i][0],
			pFrameOfData->OtherMarkers[i][2],
			pFrameOfData->OtherMarkers[i][1]
		);
	}
*/
	// Markers is now the full


}
