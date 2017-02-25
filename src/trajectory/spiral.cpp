#include <tansa/trajectory.h>

namespace tansa {


SpiralTrajectory::SpiralTrajectory(const Point &origin, double radius, double theta1, double t1, double theta2, double t2) : Trajectory(t1, t2) {
	this->origin = origin;
	this->radius = radius;
	this->theta1 = theta1;
	this->theta2 = theta2;
}


TrajectoryState SpiralTrajectory::evaluate(double t) {

	// r = a + b * theta
	// a = theta1
	// b = (radiusFinal - a) / thetaFinal
	// x = r*cos(theta)
	// y = r*sin(theta)

	double a = theta1;
	double b = (radius - a) / theta2;

	double theta = (((t - t1) / (t2 - t1)) * (theta2 - theta1)) + theta1;

	double r = a + b*theta;

	TrajectoryState s;

	// NOTE: These 3 lines are auto-generated from spiral.py
	s.position = Vector3d(r*cos(theta), r*sin(theta), 0) + origin;
	s.velocity = Vector3d(b*(-theta1 + theta2)*cos(theta)/(-t1 + t2) - r*(-theta1 + theta2)*sin(theta)/(-t1 + t2), b*(-theta1 + theta2)*sin(theta)/(-t1 + t2) + r*(-theta1 + theta2)*cos(theta)/(-t1 + t2), 0);
	s.acceleration = Vector3d(-2*b*pow(-theta1 + theta2, 2)*sin(theta)/pow(-t1 + t2, 2) - r*pow(-theta1 + theta2, 2)*cos(theta)/pow(-t1 + t2, 2), 2*b*pow(-theta1 + theta2, 2)*cos(theta)/pow(-t1 + t2, 2) - r*pow(-theta1 + theta2, 2)*sin(theta)/pow(-t1 + t2, 2), 0);

	return s;
}


}
