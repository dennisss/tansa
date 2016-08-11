
#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>
#include <mavlink.h>


ros::ServiceClient client;


void set_lights(float a, float b){
	mavros_msgs::CommandLong srv;

	srv.request.command = MAV_CMD_DO_SET_SERVO;
	srv.request.param1 = a;
	srv.request.param2 = b;

	if(client.call(srv)) {


		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call command service");
		//return 1;
	}

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "light");
	ros::NodeHandle n;

	client = n.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");



	ros::Rate rate(10);

	float level = 0;
	float dl = 0.05;


	while(ros::ok()){

		set_lights(level, level);

		level += dl;
		if(level >= 1.0 || level <= 0.0)
			dl = -dl;

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
