#ifndef TANSA_CONTROL_H_
#define TANSA_CONTROL_H_

#include "vehicle.h"
#include "trajectory.h"

/*
	The general idea is that every Vehicle will always have a Controller associated with it.
	When a trajectoy is completed,
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

	void track(Trajectory *t);


	/**
	 * This should be called 100 times a second to track the path
	 */
	void control(double t);


private:
	Vehicle *vehicle;

	PID<3> *pid;
};


#endif
