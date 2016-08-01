

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

#include <iostream>

using namespace std;


void modelstate_callback(const gazebo_msgs::ModelStates& msg){

	for(int i = 0; i < msg.name.size(); i++){
		if(msg.name[i] == "iris"){
			cout << msg.pose[i].position.x << " " << msg.pose[i].position.y << " " << msg.pose[i].position.z << endl;
		}
	}


}


/*
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Twist[] twist   # desired twist in world frame


*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_log");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, modelstate_callback);



	ros::spin();



}
