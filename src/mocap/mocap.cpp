#include <tansa/mocap.h>
#include <tansa/vehicle.h>
#include <tansa/time.h>

#include "optitrack/NatNetClient.h"


Mocap::Mocap() {


}


int Mocap::connect(string iface_addr) {
	client = new NatNetClient();

	if(client->Initialize((char *)iface_addr.c_str(), NULL) != 0){
		delete client;
		return 1;
	}

	client->SetDataCallback(data_callback, (void *) this);

	return 0;
}

int Mocap::disconnect() {


}



void data_callback(sFrameOfMocapData* pFrameOfData, void* pUserData){

	Mocap *inst = (Mocap *) pUserData;

	uint64_t t = tansa::Time::now().micros(); // TODO: Replace with the mocap timestamp

	for(int i = 0; i < pFrameOfData->nRigidBodies; i++){

		// TODO: Use the ID to look it up in the table of names from sRigidBodyDescription (replace spaces with underscores)
		// Hash map id's to their publisher

		sRigidBodyData *rb = &pFrameOfData->RigidBodies[i];

		int id = rb->ID;


		// Only continue if a key exists for this rigid body
		if(inst->tracked.count(id) == 1){
			continue;
		}


		// Don't send it if it wasn't tracked correctly
		bool bTrackingValid = rb->params & 0x01;
		if(!bTrackingValid)
			continue;


		// Conversion from default Optitrack coordinate system to ENU
		// Invert x and swap y & z
		Vector3d pos(
			-rb->x
			rb->z,
			rb->y
		);

		Quaternond quat(
			rb->qw,
			-rb->qx,
			rb->qz,
			rb->qy
		);


		inst->tracked[id]->mocap_update(pos, quat, t);
	}


/*
	tansa::PointArray markers;
	markers.points.resize(pFrameOfData->nOtherMarkers);
	for(int i = 0; i < markers.points.size(); i++){
		// TODO: I need to change coordinate systems here
		markers.points[i].x = -pFrameOfData->OtherMarkers[i][0];
		markers.points[i].y = pFrameOfData->OtherMarkers[i][2];
		markers.points[i].z = pFrameOfData->OtherMarkers[i][1];
	}
	markerPub.publish(markers);
*/

}
