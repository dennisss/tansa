#include <tansa/trajectory.h>

#include <vector>
#include <iostream>


using namespace std;

namespace tansa {

/*
	Linear trajectories go from point x1 @ t1 to point x2 @ t2 in the smoothest way possible
	The velocity and acceleration at the end points should be 0

	This constructor generates the optimal minimum jerk polynomial
*/



LinearTrajectory::LinearTrajectory(Point x1, double t1, Point x2, double t2)
 	: Trajectory(t1, t2) {

	// Constrain only initial and final positions
	inner = PolynomialTrajectory::compute({x1}, t1, {x2}, t2);
}


TrajectoryState LinearTrajectory::evaluate(double t) {
	return inner->evaluate(t);
}

}
