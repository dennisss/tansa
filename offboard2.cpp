#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pub_setpoints");
	ros::NodeHandle n;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	//ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");





	ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
	ros::Rate loop_rate(100);
	ros::spinOnce();

	geometry_msgs::PoseStamped msg;
	int count = 1;

	//PositionReciever qp;:
	//Body some_object;
	//qp.connect_to_server();


	while(ros::ok()){
		//some_object = qp.getStatus();
		// some_object.print();
		//printf("%f\n",some_object.position_x);
		msg.header.stamp = ros::Time::now();
		msg.header.seq=count;
		msg.header.frame_id = 1;
		msg.pose.position.x = 0.0;//0.001*some_object.position_x;
		msg.pose.position.y = 0.0;//0.001*some_object.position_y;
		msg.pose.position.z = 1.0;//0.001*some_object.position_z;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 0;
		msg.pose.orientation.w = 1;

		chatter_pub.publish(msg);
		ros::spinOnce();
		count++;
		loop_rate.sleep();
	}


	return 0;
}
