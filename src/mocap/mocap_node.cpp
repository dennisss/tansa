#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tansa/PointArray.h>

#include <map>
#include <string>

#include "optitrack/NatNetClient.h"


#include <iostream>

using namespace std;
using namespace geometry_msgs;


NatNetClient* client;


// For every registered ID, there should be a publisher
map<int, ros::Publisher> rigidPubs;
ros::Publisher markerPub;

void data_callback(sFrameOfMocapData* pFrameOfData, void* pUserData){

	for(int i = 0; i < pFrameOfData->nRigidBodies; i++){

		// TODO: Use the ID to look it up in the table of names from sRigidBodyDescription (replace spaces with underscores)
		// Hash map id's to their publisher

		sRigidBodyData *rb = &pFrameOfData->RigidBodies[i];


		// Don't send it if it wasn't tracked correctly
		bool bTrackingValid = rb->params & 0x01;
		if(!bTrackingValid)
			continue;





		// Currently, only publish id 1
		if(rb->ID != 1){
			continue;
		}



		PoseStamped p;

		Point *pos = &p.pose.position;
		Quaternion *quat = &p.pose.orientation;

		p.header.stamp = ros::Time::now(); // TODO: Replace with the mocap timestamp

		pos->x = -rb->x;
		pos->y = rb->z;
		pos->z = rb->y;

		quat->w = rb->qw;
		quat->x = -rb->qx;
		quat->y = rb->qz;
		quat->z = rb->qy;


		rigidPubs[1].publish(p);

	}


/*
	tansa::PointArray markers;
	markers.points.resize(pFrameOfData->nOtherMarkers);
	for(int i = 0; i < markers.size(); i++){
		// TODO: I need to change coordinate systems here
		markers.points[i].x = -pFrameOfData->OtherMarkers[i][0];
		markers.points[i].y = pFrameOfData->OtherMarkers[i][2];
		markers.points[i].z = pFrameOfData->OtherMarkers[i][1];
	}
	markerPub.publish(markers);
*/

}



int main(int argc, char *argv[]) {

	ros::init(argc, argv, "mocap_node");
	ros::NodeHandle n;

	string client_addr;
	n.getParam("client_addr", client_addr);



	client = new NatNetClient();


	// TODO: Eventually parametrize these
	if(client->Initialize((char *)client_addr.c_str(), NULL) != 0){
		return 1;
	}


	client->SetDataCallback(data_callback, NULL);


	markerPub = n.advertise<tansa::PointArray>("mocap/markers", 1000);

	rigidPubs[1] = n.advertise<geometry_msgs::PoseStamped>("mocap/1/pose", 1000);








	ROS_INFO("Mocap Ready");
	ros::spin();


	return 0;
}
