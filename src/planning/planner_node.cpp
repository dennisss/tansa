

#include <ros/ros.h>

#include <iostream>

using namespace std;


// TODO: I should also output JSON messages over stdout for communicating with the server that called it
// Once a final 'done' message has been received by the node.js server, we can 

// TODO: Instead, use the individual swarm_node poses at /swarm/1/pose

geometry_msgs::Vector3 cur_position;

void modelstate_callback(const gazebo_msgs::ModelStates& msg){

	for(int i = 0; i < msg.name.size(); i++){
		if(msg.name[i] == "iris"){
			cout << msg.pose[i].position.x << " " << msg.pose[i].position.y << " " << msg.pose[i].position.z << endl;

			cur_position = msg.pose[i].position;
		}
	}


}


int main(int argc, char *argv[]){

	ros::init(argc, argv, "planner_node");
	ros::NodeHandle nh;


	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");


	// Ensure  that all drones are connected


	// Ensure that gazebo is
	ros::service::waitForService("/gazebo/set_model_state");






	// Initialize all output groups
		// Switch all drones to POSCTL (so that they hover when losing a setpoint)
		// Start sending the home position as a setpoint
		// Switch to OFFBOARD

	// Maintain all outputs groups in a separate thread (continuously send setpoints in another thread)




	// Listen to current state
	ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1, modelstate_callback);



	float points[] = {
		0,0,2,
		1,0,2,
		1,1,2,
		0,1,2,
		0,0,2
	};

	int i = 0;
	int npoints = 5;



	ros::Rate r(60); // 60 hz
	while (ros::ok()){


		// Perform update for current plan base don current inputs
			// It should update the output



		// Set outputs
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = points[i*3];
		pose.pose.position.y = points[i*3 + 1];
		pose.pose.position.z = points[i*3 + 2];
		local_pos_pub.publish(pose);


		// Proceed to next point once close enough
		// TODO: Also have a timeout if minimal progress isn't made in 1 second
		if(error < 0.1){
			i++;
		}




		// Done all points
		if(i >= npoints)
			break;

		r.sleep();
	}

	cout << "Done all points" << endl;

}


void run(){

	// Start of planner time
	ros::Time begin = ros::Time::now();





}
