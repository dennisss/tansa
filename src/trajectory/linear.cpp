#include <tansa/trajectory.h>

/*
	Linear trajectories go from point x1 @ t1 to point x2 @ t2 in the smoothest way possible

	The velocity and acceleration at the end points should be 0
	Although not optimal, we use a quadratic to describe velocity and a cubic to describe position. Acceleration is held at 0.

	Position math: (note: time and position are normalized from 0 to 1)
	x = (t - 1)^3 + 1
	@ t = 0, x = 0
	@ t = 0, x = 1

	Velocity:
	v = 3 * (t - 1)^2
	@ t = 0, v = 3

*/


Vector4d LinearTrajectory::position(double t) {
	double tN = (t - t1) / (t2 - t1);
	double a = tN - 1.0;
	Vector4d x = a*a*a + 1.0;
	return x1 + (x2 - x1).cwiseProduct(x);
}


Vector4d LinearTrajectory::velocity(double t) {
	double tN = (t - t1) / (t2 - t1);
}

Vector4d LinearTrajectory::acceleration(double t) {
	return Vector4d::Zero();
}
