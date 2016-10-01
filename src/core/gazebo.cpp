/*
	Helpers for synchronizing the program with gazebo (in particular for timing and position purposes)
*/

#include <tansa/core.h>
#include <tansa/time.h>
#include <tansa/vehicle.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>


// TODO: These need to be cleaned up
static gazebo::transport::NodePtr node;
static gazebo::transport::SubscriberPtr world_sub;
static gazebo::transport::SubscriberPtr poses_sub;
static Vehicle *v;

// TODO: These also come with a timestamp, so we might as well use it
void gazebo_poses_callback(ConstPosesStampedPtr &posesStamped) {

	for(int i = 0; i < posesStamped->pose_size(); ++i) {

		const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
		std::string name = pose.name();
		if(name == std::string("iris")) {
			const ::gazebo::msgs::Vector3d &position = pose.position();
			const ::gazebo::msgs::Quaternion &orientation = pose.orientation();

			Vector3d pos(
				position.x(),
				position.y(),
				position.z()
			);

			Quaterniond orient(
				orientation.w(),
				orientation.x(),
				orientation.y(),
				orientation.z()
			);

			// Not needed as the simulation has ground truth odometry
		//	if(v != NULL) {
		//		v->mocap_update(pos, orient, 0);
		//	}

		}

	}

	// std::cout << msg->DebugString();
}

bool gazebo_registered = false;
Time gazebo_last_time(0,0);

void gazebo_stats_callback(ConstWorldStatisticsPtr &msg) {
	Time t(msg->sim_time().sec(), msg->sim_time().nsec());
	Time rt(msg->real_time().sec(), msg->real_time().nsec());

//	Time rt = Time::realNow();
//	if(gazebo_registered) {
	Time::setTime(t, t.seconds() / rt.seconds());
//	}

//	gazebo_last_time =
}


void sim_init() {
	v = NULL;
	node = NULL;
}

namespace tansa {

void sim_connect() {

	// Hold time at 0 until it is initialized
	Time::setTime(Time(0,0), 0);

	printf("Connecting to gazebo...\n");

	// Load gazebo
	gazebo::client::setup();

	// Create our node for communication
	node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	node->Init();

	world_sub = node->Subscribe("~/world_stats", gazebo_stats_callback);
	poses_sub = node->Subscribe("~/pose/info", gazebo_poses_callback);

	printf("- done\n");

	// Busy wait loop...replace with your own code as needed.
	//while (true)
	//	gazebo::common::Time::MSleep(10);
}


void sim_disconnect() {
	// Make sure to shut everything down.
	gazebo::client::shutdown();

	// Deleting reference to old Boost pointer
	world_sub = gazebo::transport::SubscriberPtr();
	poses_sub = gazebo::transport::SubscriberPtr();
}

void sim_track(Vehicle *veh) {
	v = veh;
}

}

/*
This is what the data looks like:

/gazebo/default/world_stats
Type: gazebo.msgs.WorldStatistics

time {
  sec: 401
  nsec: 656000000
}


/gazebo/default/pose/info
Type: gazebo.msgs.PosesStamped

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
