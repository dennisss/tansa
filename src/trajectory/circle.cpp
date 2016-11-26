#include <tansa/trajectory.h>

namespace tansa {

CircleTrajectory::CircleTrajectory(const Point &origin, double radius, double theta1, double t1, double theta2, double t2)
	: Trajectory(t1, t2) {

	this->origin = origin;
	this->radius = radius;

	this->theta1 = theta1;
	this->dtheta = (theta2 - theta1) / (t2 - t1);
}


TrajectoryState CircleTrajectory::evaluate(double t) {

	TrajectoryState s;

	double theta = ((t - t1) * dtheta) + theta1; // angle as a function of time

	s.position = Point(radius*sin(theta), radius*cos(theta), 0) + origin; //1 + t / 10.0);
	s.velocity = Point(radius*dtheta*cos(theta), -radius*dtheta*sin(theta), 0); //1.0 / 10.0);
	s.acceleration = Point(-radius*dtheta*dtheta*sin(theta), -radius*dtheta*dtheta*cos(theta), 0);

	return s;
}

}
