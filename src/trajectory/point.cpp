
#include <tansa/trajectory.h>


PointTrajectory::PointTrajectory(const Point &p)
	: Trajectory(0, 1000000) {

	this->p = p;
}


TrajectoryState PointTrajectory::evaluate(double t) {

	TrajectoryState s;

	s.position = p;
	s.velocity = Point::Zero();
	s.acceleration = Point::Zero();

	return s;
}
