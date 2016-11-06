#include <tansa/estimation.h>

/*
	Simple state estimator for linear parts of motion
	This is mainly used for latency compensation and filtering of the raw mocap data
*/

void LinearComplementaryEstimator::predict(State &s, ControlInput u, const Time &t) {
	double dt = t.since(s.time).seconds();

	Vector3d a_xyz = u.segment<3>(0);

	Vector3d x = (dt*dt / 2.0) * a_xyz + dt * s.velocity + s.position;
	Vector3d v =  dt * a_xyz + s.velocity;

	s.position = x;
	s.velocity = v;
	s.time = t;
}

void LinearComplementaryEstimator::correct(State &s, const Vector3d &x, const Vector3d &v, const Time &t) {

	double cP = 0.99;
	double cV = 0.80;

	s.position = cP*x + (1.0 - cP)*s.position;
	s.velocity = cV*v + (1.0 - cV)*s.velocity;
}
