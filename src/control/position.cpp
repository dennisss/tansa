#include <tansa/control.h>

#include "pid.h"

namespace tansa {

// TODO: Integrate forward the position, velocity and time based on connection latency (so that commmands are accurate for the cmoment at which they are received)

PositionController::PositionController(Vehicle *v, bool directAttitudeControl) {
	this->vehicle = v;

	Vector3d windupMin = Point(-2.0, -2.0, -2.0),
			 windupMax = Point(2.0, 2.0, 2.0);


	PID<PointDims>::State::Ptr s = std::make_shared<PID<PointDims>::State>();

	pid = new PID<PointDims>(s);
	pid->setWindupOutputLimit(windupMin, windupMax);
	pid->setGains(
		v->params.gains.p,
		Vector3d(0, 0, v->params.gains.i.z()), // Only perform z integration for position control
		//Point::Zero(), // Don't use
		v->params.gains.d
	);


	hoverPid = new PID<PointDims>(s);
	hoverPid->setWindupOutputLimit(windupMin, windupMax);
	hoverPid->setGains(
		v->params.gains.p,
		v->params.gains.i,
		v->params.gains.d
	);


	this->directAttitudeControl = directAttitudeControl;
}

void PositionController::track(Trajectory::Ptr traj) {
	this->trajectory = traj;
	hoverMode = false;
}

void PositionController::track(Point point) {
	this->point = point;
	hoverMode = true;
}

TrajectoryState PositionController::getTargetState(double t) {
	if(hoverMode) {
		TrajectoryState s;
		s.position = point;
		s.velocity = Point::Zero();
		s.acceleration = Point::Zero();
		return s;
	}

	return trajectory->evaluate(t);
}

void PositionController::control(double t) {
	// Evaluate trajectory
	TrajectoryState s = this->getTargetState(t);
	ModelState cur = vehicle->arrival_state();


	Vector3d eP = s.position - cur.position;
	Vector3d eV = s.velocity - cur.velocity;

	// At low altitudes, there's no room to rotate so we reduce the error effect
	// TODO: Have a better way of doing this: essentially we want a trajectory straight up followed
	if(s.position.z() < 0.1) {
		double factor = 0.8;
		for(unsigned i = 0; i < 2; i++) {
			eP(i) *= factor;
			eV(i) *= factor;
		}
	}

	Vector3d out;
	if(hoverMode) {
		// Do nothing if hovering on the ground
		if(point.z() < 0.1) {
			vehicle->setpoint_zero();
			return;
		}

		out = hoverPid->compute(eP, eV, 0.01);
	}
	else {
		out = pid->compute(eP, eV, 0.01 /* TODO: Make this more dynamic */);
	}

	Vector3d a = out + s.acceleration;

	double yaw_angle = DEFAULT_YAW_ANGLE;

	if(directAttitudeControl) {
		vehicle->lastControlInput = a;

		a += Vector3d(0, 0, GRAVITY_MS);

		Quaterniond att = Quaterniond::FromTwoVectors(Vector3d(0,0,1), a.normalized());
		Quaterniond yaw( AngleAxisd(yaw_angle, Vector3d::UnitZ()) );
		vehicle->setpoint_attitude(att*yaw, a.norm());
	}
	else {
		vehicle->setpoint_accel(a, yaw_angle);
	}
}

}
