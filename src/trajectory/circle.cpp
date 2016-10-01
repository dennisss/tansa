#include <tansa/trajectory.h>


/*
	Circles are characterized by:
	- radius
	- normal vector
	- center
	- start angle  and start time
	- end angle  and end time

*/


TrajectoryState CircleTrajectory::evaluate(double t) {

	TrajectoryState s;

	double r = 2;
	double dTheta = 40.0 * M_PI / 180.0; // radians per second
	double theta = t * dTheta; // angle as a function of time
	s.position = Point(r*sin(theta), r*cos(theta), 1); //1 + t / 10.0);
	s.velocity = Point(r*dTheta*cos(theta), -r*dTheta*sin(theta), 0); //1.0 / 10.0);
	s.acceleration = Point(-r*dTheta*dTheta*sin(theta), -r*dTheta*dTheta*cos(theta), 0);

	return s;
}
