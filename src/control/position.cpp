#include <tansa/control.h>

#include "pid.h"

// TODO: Integrate forward the position, velocity and time based on connection latency (so that commmands are accurate for the cmoment at which they are received)

PositionController::PositionController(Vehicle *v) {
	this->vehicle = v;

	pid = new PID<PointDims>();

	pid->setGains(
		v->params.gains.p,
		Point::Zero(), // Don't use
		v->params.gains.d
	);
}

void PositionController::track(Trajectory *traj) {
	this->trajectory = traj;
}

TrajectoryState PositionController::getTargetState(double t) {
	return trajectory->evaluate(t);
}

void PositionController::control(double t) {
	// Evaluate trajectory
	TrajectoryState s = this->getTargetState(t);
	State cur = vehicle->arrival_state();


	Vector3d eP = s.position - cur.position;
	Vector3d eV = s.velocity - cur.velocity;

	Vector3d a = pid->compute(eP, eV, 0.01 /* TODO: Make this more dynamic */) + s.acceleration;

	vehicle->setpoint_accel(a);
}
