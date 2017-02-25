#ifndef TANSA_ACTION_H
#define TANSA_ACTION_H

#include "trajectory.h"
#include <memory>

namespace tansa {

/**
 * Type representing the different actions in jocs
 * @enum
 */
enum ActionTypes : unsigned{
	None = 0,
	Transition = 1,
	Line = 2,
	Circle = 3,
	Hover = 4,
	Light = 5,
	Strobe = 6,
	GradualCricle = 7
};

typedef unsigned DroneId;
/**
 * @enum Defines the different lights on the drone
 */
//typedef unsigned LightId;
/**
 * @interface Defines an interface for all Actions to implement
 */
class Action {
public:
	/**
	 * @param id Id of the drone this action refers to
	 */
	Action(DroneId id, ActionTypes type) : droneId(id), type(type) {}
	virtual ~Action(){}
	/**
	 * @virtual
	 * @return Double representing the seconds after initialization that this action should start at
	 */
	virtual double GetStartTime() const = 0;
	/**
	 * @virtual
	 * @return Double representing the seconds after initialization that this action should end at
	 */
	virtual double GetEndTime() const = 0;

	/**
	 * @return Id of drone this action refers to
	 */
	inline DroneId GetDrone() { return droneId; }

	/**
	 * @return false if action needs to be calculated in post-process step
	 */
	inline bool IsCalculated() { return isCalculated; }
	inline ActionTypes GetActionType() { return type; }

protected:
	DroneId droneId;
	ActionTypes type;
	bool isCalculated = false;
};
/**
 * @class Represents an action to be replaced by another action (usually for transitions)
 */
class EmptyAction : public Action {
public:
    EmptyAction(DroneId id, double s, double e): Action(id,ActionTypes::Transition), startTime(s), endTime(e){}
	virtual ~EmptyAction(){}
    virtual double GetStartTime() const { return startTime; }

    virtual double GetEndTime() const { return endTime; }



private:
    double startTime;
    double endTime;
};
/**
 * @class Represents an action with a trajectory. Trajectories can be of different types and result in different
 * types of motion actions
 */
class MotionAction : public Action {
public:
	MotionAction(DroneId id, Trajectory::Ptr t, ActionTypes type) : Action(id, type), path(t) { isCalculated = true; }
	virtual ~MotionAction(){ }

	inline TrajectoryState GetPathState(double t){ return path->evaluate(t); }

	inline Trajectory::Ptr GetPath(){return path;}

	virtual double GetStartTime() const { return path->startTime(); }

	virtual double GetEndTime() const { return path->endTime(); }

	inline Point GetStartPoint() const { return path->evaluate(path->startTime()).position; }
	inline Point GetEndPoint() const { return path->evaluate(path->endTime()).position; }

private:
	Trajectory::Ptr path;
};

/**
 * @class Represents a light action (turning light on or off, patterns, etc)
 */
class LightAction : public Action {
public:
	LightAction(DroneId did, LightTrajectory* t) :
			Action(did, ActionTypes::Light), path(t)
			{ isCalculated = true; }
	virtual ~LightAction(){ delete path; }

	inline double GetPathState(double t) { return path->evaluate(t); }

	inline LightTrajectory* GetPath() { return path; }

	virtual double GetStartTime() const { return path->getStartTime(); }
	virtual double GetEndTime() const { return path->getEndTime(); }

	/**
	 * @return Current set intensity of the light
	 */
	inline double GetStartIntensity() { return path->evaluate(path->getStartIntensity()); }
	inline double GetEndIntensity() { return path->evaluate(path->getEndIntensity()); }

	/**
	 * @return Which light this action is referring to.
	 */
	//inline LightId GetLightId() { return lightId; }

private:
	LightTrajectory* path;
	//LightId lightId;
};

}

#endif //TANSA_ACTION_H
