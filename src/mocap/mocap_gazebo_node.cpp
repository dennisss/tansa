/*
	Uses gazebo models for generating mocap data
*/

#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>

static ros::Subscriber sub;
static ros::Publisher pub;

#include <iostream>

using namespace std;


// This will be called at around 500hz, so we will need to reduce the rate to around 120hz for a more realistic simulation
void modelstate_callback(const gazebo_msgs::ModelStates& msg){

	for(int i = 0; i < msg.name.size(); i++){

		geometry_msgs::PoseStamped p;

		// This will use the simulation time
		p.header.stamp = ros::Time::now();
		p.pose = msg.pose[i];

		if(msg.name[i] == "iris"){
			cout << msg.pose[i].position.x << " " << msg.pose[i].position.y << " " << msg.pose[i].position.z << endl;
			pub.publish(p);
			break;
		}
	}


}


int main(int argc, char *argv[]){

	ros::init(argc, argv, "mocap_gazebo_node");

	ros::NodeHandle nh;


	sub = nh.subscribe("/gazebo/model_states", 1, modelstate_callback);
	pub = nh.advertise<geometry_msgs::PoseStamped>("/mocap/1/pose", 1000); // "/swarm/1/mavros/mocap/pose"

}
