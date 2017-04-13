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
	/**
	 * Creates a position controller for a vehicle
	 *
	 * @param v
	 * @param directAttitudeControl (optional) setting to true will directly control the attitude of the drone: this bypasses PX4 position controller safety features but is requested for close to inverted acrobatics. otherwise, acclerations will be fed to PX4
	 */
	PositionController(Vehicle *v, bool directAttitudeControl = true);
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
	bool directAttitudeControl;
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

	enum LightIndices {
		LEFT = 0,
		RIGHT = 1,
		INTERNAL = 2, 
	};
	static const int NUM_LIGHTS = 3;
	static const int MAX_LIGHTS = 7;

	LightController(Vehicle *v);

	/**
	 * Specifies which trajectory should be followed for which light
	 */
	void track(LightTrajectory::Ptr traj, LightIndices index) { trajectories[index] = traj; }


	/**
	 * This should be called 100 times a second to track the path
	 */
	virtual void control(double t);

private:
	double EPSILON = 0.01;
	std::vector<int> lightStates;
	Vehicle *vehicle;
	LightTrajectory::Ptr trajectories[NUM_LIGHTS];
};

}

#endif
