#include <tansa/control.h>

#include "pid.h"

// TODO: Integrate forward the position, velocity and time based on connection latency (so that commmands are accurate for the cmoment at which they are received)

LightController::LightController(Vehicle *v) {
	this->vehicle = v;
}

void LightController::track(LightTrajectory *traj) {
	this->trajectory = traj;
}

void LightController::control(double t) {

	// Evaluate trajectory
	double s = trajectory->evaluate(t);

	vehicle->set_lighting(s, s);

	printf("Light: %.2f at %.2f\n", s, t);
}
