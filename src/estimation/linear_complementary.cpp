#include <tansa/estimation.h>

/*
	Simple state estimator for linear parts of motion
	This is mainly used for latency compensation and filtering of the raw mocap data
*/

void LinearComplementaryEstimator::predict(State &s, ControlInput u, const Time &t) {
	double dt = t.since(s.time).seconds();

	Vector3d a_xyz = u.segment<3>(0);

	Vector3d x = (dt*dt / 2.0) * a_xyz + dt * v.velocity + s.position;
	Vector3d v =  dt * a_xyz + v.velocity;

	s.position = x;
	s.velocity = v;
	s.time = t;
}

void correct(const State &s, Vector3d x, Vector3d v) {
	Vector3d c(
		0.90,
		0.90,
		0.90
	);

	Vector3d cm = Vector3d::Ones() - c;

	s.position = c*x + cm*s.position;
	s.velocity = c*v + cm*s.velocity;
}
