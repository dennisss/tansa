#include <tansa/trajectory.h>


namespace tansa {

EllipseTrajectory::EllipseTrajectory(const Point &origin, double radius_x, double radius_y, double theta1, double t1, double theta2, double t2)
	: Trajectory(t1, t2) {

	this->origin = origin;
	this->radius_x = radius_x;
	this->radius_y = radius_y;

	this->theta1 = theta1;
	this->dtheta = (theta2 - theta1) / (t2 - t1);
}


TrajectoryState EllipseTrajectory::evaluate(double t) {

	TrajectoryState s;

	double theta = ((t - t1) * dtheta) + theta1; // angle as a function of time

	s.position = Point(radius_x*sin(theta), radius_y*cos(theta), 0) + origin; //1 + t / 10.0);
	s.velocity = Point(radius_x*dtheta*cos(theta), -radius_y*dtheta*sin(theta), 0); //1.0 / 10.0);
	s.acceleration = Point(-radius_x*dtheta*dtheta*sin(theta), -radius_y*dtheta*dtheta*cos(theta), 0);

	return s;
}

}
