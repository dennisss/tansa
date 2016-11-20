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

	double hover = vehicle->params.hoverPoint;

	// Evaluate trajectory
	TrajectoryState s = this->getTargetState(t);
	State cur = vehicle->arrival_state();


	Vector3d eP = s.position - cur.position;
	Vector3d eV = s.velocity - cur.velocity;

	Vector3d a = pid->compute(eP, eV, 0.01 /* TODO: Make this more dynamic */) + s.acceleration;

	// Scale to -1 to 1 range and add hover point because PX4 doesn't take m s^-2 input but rather input proportional to thrust percentage
	a = a / (9.8 / hover) + Vector3d(0, 0, hover);

	vehicle->setpoint_accel(a);

	printf("A: %.2f %.2f %.2f  E: %.2f\n", a.x(), a.y(), a.z(), eP.norm());
}
