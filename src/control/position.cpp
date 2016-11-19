#include <tansa/control.h>

#include "pid.h"

// TODO: Integrate forward the position, velocity and time based on connection latency (so that commmands are accurate for the cmoment at which they are received)

PositionController::PositionController(Vehicle *v) {
	this->vehicle = v;

	pid = new PID<3>();

	pid->setGains(
		Point(3.4, 3.4, 3.4), // p
		Point(0, 0, 0), // i
		Point(1.0, 1.0, 1.0) // d
	);
}

void PositionController::track(Trajectory *traj) {
	this->trajectory = traj;
}

void PositionController::control(double t) {

	//printf("%.2f\n", t);

	double hover = 0.45; // 0.51

	// Evaluate trajectory
	TrajectoryState s = trajectory->evaluate(t);

	Vector3d eP = s.position - vehicle->state.position;
	Vector3d eV = s.velocity - vehicle->state.velocity;

	Vector3d a = pid->compute(eP, eV, 0.01 /* TODO: Make this more dynamic */) + s.acceleration;

	// Scale to -1 to 1 range and add hover point because PX4 doesn't take m s^-2 input but rather input proportional to thrust percentage
	// TODO: PX4 should handle battery percentage and internal resistance calibration
	a = a / (9.8 / hover) + Vector3d(0, 0, hover);

	vehicle->setpoint_accel(a);

	//printf("A: %.2f %.2f %.2f  E: %.2f\n", a.x(), a.y(), a.z(), eP.norm());
}
