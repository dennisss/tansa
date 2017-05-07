
#include <tansa/gazebo.h>

using namespace tansa;


int main(int argc, char *argv[]) {

	GazeboConnector con;

	// This will stale until gazebo is started
	con.connect();

	vector<Vector3d> homes;
	homes.push_back({0, 0, 0});
	homes.push_back({0, 1, 0});

	// After calling this, we can connect to each drone by listening on port 14550 and 14560 (14550 + 10*i)
	con.spawn(homes, "config/gazebo/models/x340/x340.sdf", "config/models/x340/rcS");

	con.disconnect();

}
