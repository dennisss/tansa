#include <tansa/control.h>

namespace tansa {

#include "pid.h"


HoverController::HoverController(Vehicle *v) : PositionController(v) {
	this->point = Point(0,0,0);

	// Position controller gains + integral term
	pid->setGains(
		v->params.gains.p,
		v->params.gains.i,
		v->params.gains.d
	);

	pid->setWindupOutputLimit(
		Point(-2.0, -2.0, -2.0),
		Point(2.0, 2.0, 2.0)
	);
}

TrajectoryState HoverController::getTargetState(double t) {
	TrajectoryState s;
	s.position = point;
	s.velocity = Point::Zero();
	s.acceleration = Point::Zero();
	return s;
}

void HoverController::control(double t) {

	// If really low to the ground, do nothing
	if(point.z() < 0.1) {
		vehicle->setpoint_zero();
		return;
	}

	PositionController::control(t);

	/*
	// This is already built into PX4, so we will let it handle staying in one spot
	// Precision over speed should considered when calibrating the PX4 position controller

	vehicle->setpoint_pos(point);
	*/
}

double HoverController::distance() {
	VectorXd e(6);
	e << (point - vehicle->state.position),
	     (Vector3d(0,0,0) - vehicle->state.velocity);

	return e.norm();
}

}
