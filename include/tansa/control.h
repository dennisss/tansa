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
 * Controller for tracking orientation setpoints
 *
 * This is mainly used for simulation right now, and is setup to have similar behavior to the current PX4 P-PID loops
 * The outer loop is a P regulator of angle with gains with units (rad/s for 1 rad error)
 * The inner loop is a PID regulator of angular rate with units (control output for 1 rad/s error)
 * The angular outputs are in the range [-1 to 1]
 * Thrust is a passthrough in the range of [0 to 1]
 */
class AttitudeController : public Controller {
public:

	AttitudeController(Vehicle::Ptr v);


	void control_angles(Quaterniond q, double thrust);
	//void control_rates(Quaterniond q, )


private:
	PID<3> *angle_system;
	PID<3> *rates_system;


};

// By default face in the -y direction (in ENU)
#define DEFAULT_YAW_ANGLE (-M_PI / 2)

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
	PositionController(Vehicle::Ptr v, bool directAttitudeControl = true);
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
	Vehicle::Ptr vehicle;

	PID<PointDims> *pid;

private:
	Trajectory::Ptr trajectory;
	bool directAttitudeControl;
};


class HoverController : public PositionController {
public:
	HoverController(Vehicle::Ptr v);
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


/**
 * LQR(I) controller for following trajectories using angular rate controls
 */
class LQRController : public Controller {
	LQRController(Vehicle::Ptr v);
	virtual ~LQRController() {}


	void track(Trajectory::Ptr traj);

	virtual TrajectoryState getTargetState(double t);

	virtual void control(double t);

private:

	MatrixXd K;

};


class AdmittanceController : public HoverController {
public:
	AdmittanceController(Vehicle::Ptr v);
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

	LightController(Vehicle::Ptr v);

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
	Vehicle::Ptr vehicle;
	LightTrajectory::Ptr trajectories[NUM_LIGHTS];
};

}

#endif
