#ifndef TANSA_CONTROL_H_
#define TANSA_CONTROL_H_

#include "vehicle.h"
#include "trajectory.h"

namespace tansa {

/*
	The general idea is that every Vehicle will always have some Controllers associated with it.
	When a trajectoy is completed, the controller may change slightly or another controller may take over
*/

/**
 * Over time regulates the behavior of something: usually the vehicles motion or lighting
 */
class Controller {
public:

	virtual void control(double t) = 0;

};

template<unsigned int N> class PID;

/**
 * Controller for following Trajectory's
 */
class PositionController : public Controller {
public:
	PositionController(Vehicle *v);
	virtual ~PositionController() {}

	/**
	 * Specifies which trajectory should do followed
	 */
	void track(Trajectory::Ptr traj);

	virtual TrajectoryState getTargetState(double t);

	/**
	 * This should be called 100 times a second to track the path
	 */
	virtual void control(double t);

protected:
	Vehicle *vehicle;

	PID<PointDims> *pid;

private:
	Trajectory::Ptr trajectory;

};


class HoverController : public PositionController {
public:
	HoverController(Vehicle *v);
	virtual ~HoverController() {}

	void setPoint(const Point &p) { this->point = p; }
	Point getPoint() { return this->point; }

	virtual TrajectoryState getTargetState(double t);

	virtual void control(double t);

	/**
	 * Get the distance to the point being kept
	 */
	double distance();

private:
	Point point;
};


class AdmittanceController : public HoverController {
public:
	AdmittanceController(Vehicle *v);
	virtual ~AdmittanceController() {}

	virtual void control(double t);

};

/**
 * Controller for following Light Trajectory's
 */
class LightController : public Controller {
public:
	LightController(Vehicle *v);

	/**
	 * Specifies which trajectory should do followed
	 */
	void track(LightTrajectory *trajTop, LightTrajectory *trajBot);


	/**
	 * This should be called 100 times a second to track the path
	 */
	virtual void control(double t);

private:
	double EPSILON = 0.01;

	Vehicle *vehicle;
	LightTrajectory *trajectoryTop;
	LightTrajectory *trajectoryBot;
	double currentIntensityTop = 0.0;
	double currentIntensityBot = 0.0;
};

}

#endif
