#ifndef TANSA_CONTROL_H_
#define TANSA_CONTROL_H_

#include "vehicle.h"
#include "trajectory.h"

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
	void track(Trajectory *traj);

	virtual TrajectoryState getTargetState(double t);

	/**
	 * This should be called 100 times a second to track the path
	 */
	virtual void control(double t);

protected:
	Vehicle *vehicle;

	PID<PointDims> *pid;

private:
	Trajectory *trajectory;

};


class HoverController : public PositionController {
public:
	HoverController(Vehicle *v);
	virtual ~HoverController() {}

	void setPoint(const Point &p) { this->point = p; }

	virtual TrajectoryState getTargetState(double t);

	virtual void control(double t);

	/**
	 * Get the distance to the point being kept
	 */
	double distance();

private:
	Point point;
};


#endif
