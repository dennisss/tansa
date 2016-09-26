/*
	Helpers for synchronizing the program with gazebo (in particular for timing and position purposes)
*/

#include <tansa/time.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>


/*
$ gz topic -i /gazebo/default/pose/info
Type: gazebo.msgs.PosesStamped

$ gz topic -i /gazebo/default/model/info
Type: gazebo.msgs.Model
*/

#include <iostream>

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstWorldStatisticsPtr &_msg)
{
	// Dump the message contents to stdout.
	std::cout << _msg->DebugString();
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
	// Load gazebo
	gazebo::client::setup(_argc, _argv);

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Listen to Gazebo world_stats topic
	gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);

	// Busy wait loop...replace with your own code as needed.
	while (true)
		gazebo::common::Time::MSleep(10);

	// Make sure to shut everything down.
	gazebo::client::shutdown();
}

/*
time {
  sec: 401
  nsec: 656000000
}
pose {
  name: "iris"
  id: 9
  position {
    x: -0.024945740985096335
    y: -0.11384451968496838
    z: 0.1048481907237855
  }
  orientation {
    x: 0.00023967189251883982
    y: -4.1931681713546753e-05
    z: 0.0046669999686749725
    w: -0.999989079895581
  }
}
*/
