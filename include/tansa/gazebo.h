#ifndef TANSA_GAZEBO_H_
#define TANSA_GAZEBO_H_

#include "vehicle.h"
#include "trajectory.h"

#include <map>
#include <vector>

/**
 * For connecting to a running Gazebo simulation
 */
class GazeboConnector {
public:
	GazeboConnector();

	void connect();
	void disconnect();
	void track(Vehicle *v, int id);

	/**
	 * Places drone models into the simulation environment at the given positions
	 */
	void spawn(const vector<Point> &homes);

};



#endif
