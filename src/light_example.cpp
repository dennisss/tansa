
#include <ros/ros.h>
#include <mavros_msgs/CommandLong.h>
#include <mavlink.h>


int main(int argc, char **argv) {

	ros::init(argc, argv, "light");
	ros::NodeHandle n;


	ros::Rate rate(10);

	float level = 0;
	float dl = 0.05;


	while(ros::ok()){

		ROS_INFO("SET %.2f", level);

		set_lights(level, level);

		level += dl;
		if(level >= 1.0 || level <= 0.0)
			dl = -dl;

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
