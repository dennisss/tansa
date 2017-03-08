/*
	Helpers for synchronizing the program with gazebo (in particular for timing and position purposes)
*/

#include <tansa/gazebo.h>
#include <tansa/core.h>
#include <tansa/time.h>
#include <tansa/vehicle.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "../../build/src/gazebo/msgs/spawn.pb.h"

#include <string.h>

#include <map>


namespace tansa {

static gazebo::transport::NodePtr node;
static gazebo::transport::SubscriberPtr world_sub;
static gazebo::transport::SubscriberPtr poses_sub;
static gazebo::transport::PublisherPtr spawn_pub;

static std::map<int, Vehicle *> tracked;

// TODO: This needs to use the class's tracked variable
// TODO: These also come with a timestamp, so we might as well use it
void gazebo_poses_callback(ConstPosesStampedPtr &posesStamped) {

	for(int i = 0; i < posesStamped->pose_size(); ++i) {
		const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
		std::string name = pose.name();

		int id = -1;
		if(strncmp(name.c_str(), "vehicle_", 8) == 0 && name.length() == 9) {
			id = atoi(name.c_str() + 8);
		}


		if(tracked.count(id) == 0){
			continue;
		}


		Vehicle *v = tracked[id];



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

		if(v != NULL) {
			v->mocap_update(pos, orient, Time::now()); // TODO: Use sim time
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


GazeboConnector::GazeboConnector() {
	node = NULL;
}


void GazeboConnector::connect() {

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

	spawn_pub = node->Advertise<tansa::msgs::SpawnRequest>("~/spawn");
	spawn_pub->WaitForConnection();


}


void GazeboConnector::disconnect() {
	// Make sure to shut everything down.
	gazebo::client::shutdown();

	// Deleting reference to old Boost pointer
	world_sub = gazebo::transport::SubscriberPtr();
	poses_sub = gazebo::transport::SubscriberPtr();
	spawn_pub = gazebo::transport::PublisherPtr();
}

void GazeboConnector::clear() {
	tracked.clear();
}

void GazeboConnector::track(Vehicle *v, int id) {
	tracked[id] = v;
}

void GazeboConnector::spawn(const vector<Point> &homes) {

	tansa::msgs::SpawnRequest req;

	for(int i = 0; i < homes.size(); i++) {
		tansa::msgs::SpawnRequest_Vehicle *v = req.add_vehicles();
		v->set_id(i);
		gazebo::msgs::Vector3d *pos = v->mutable_pos();
		gazebo::msgs::Vector3d *orient = v->mutable_orient();
		pos->set_x(homes[i].x());
		pos->set_y(homes[i].y());
		pos->set_z(homes[i].z());

		orient->set_x(0);
		orient->set_y(0);
		orient->set_z(0);
	}

	spawn_pub->Publish(req);
}

}
