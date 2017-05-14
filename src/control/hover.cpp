#include <tansa/control.h>

#include "pid.h"


namespace tansa {

HoverController::HoverController(Vehicle *v, PositionController *base) : PositionController(v) {
	this->point = Point(0,0,0);

	// Share the same state as the position controller
	delete pid->state;
	pid->state = base->pid->state;

	// Position controller gains + integral term
	pid->setGains(
		v->params.gains.p,
		v->params.gains.i,
		v->params.gains.d
	);
}

TrajectoryState HoverController::getTargetState(double t) {

}

void HoverController::control(double t) {

	// If really low to the ground, do nothing
	
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
