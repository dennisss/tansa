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

	vector<Point> c1(1);
	c1[0] = x1;

	vector<Point> c2(1);
	c2[0] = x2;

	inner = PolynomialTrajectory::compute(c1, t1, c2, t2);
}

LinearTrajectory::~LinearTrajectory() {
	delete inner;
}


TrajectoryState LinearTrajectory::evaluate(double t) {
	return inner->evaluate(t);
}

}
