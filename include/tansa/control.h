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


	/**
	 * This should be called 100 times a second to track the path
	 */
	virtual void control(double t);

private:
	Vehicle *vehicle;
	Trajectory *trajectory;

	PID<3> *pid;
};


class HoverController : public Controller {
public:
	HoverController(Vehicle *v);
	virtual ~HoverController() {}

	virtual void control(double t);

	void setPoint(const Point &p) { this->point = p; }

	/**
	 * Get the distance to the point being kept
	 */
	double distance();

private:
	Vehicle *vehicle;
	Point point;

};


#endif
