//
// Created by TessD on 10/14/2016.
//

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
	Transition = 0,
	Line = 1,
	Circle = 2,
	Hover = 3,
	Light = 4,
};

typedef unsigned DroneId;
/**
 * @enum Defines the different lights on the drone
 */
enum class LightId {TOP, BOT};
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
	ActionTypes type;
	DroneId droneId;
	bool isCalculated = false;
};
/**
 * @class Represents an action to be replaced by another action (usually for transitions)
 */
class EmptyAction : public Action {
public:
    EmptyAction(DroneId id, double s, double e): Action(id,ActionTypes::Transition), startTime(s), endTime(e){}

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
	MotionAction(DroneId id, Trajectory* t, ActionTypes type) : Action(id, type), path(t) { isCalculated = true; }
	virtual ~MotionAction(){ delete path; }

	inline TrajectoryState GetPathState(double t){ return path->evaluate(t); }

	inline Trajectory* GetPath(){return path;}

	virtual double GetStartTime() const { return path->startTime(); }

	virtual double GetEndTime() const { return path->endTime(); }

private:
	Trajectory* path;
};

/**
 * @class Represents a light action (turning light on or off, patterns, etc)
 */
class LightAction : public Action {
public:
	LightAction(double s, double e, DroneId id, double i, LightId l) :  Action(id, ActionTypes::Light),
			startTime(s), endTime(e), intensity(i), lightId(l) { isCalculated = true; }

	virtual double GetStartTime() { return startTime; }

	virtual double GetEndTime() { return endTime; }

	/**
	 * @return Current set intensity of the light
	 */
	inline double GetIntensity() { return intensity; }

	/**
	 * @return Which light this action is referring to.
	 */
	inline LightId GetLightId() { return lightId; }

private:
	double startTime;
	double endTime;
	double intensity;
	LightId lightId;
};

}

#endif //TANSA_ACTION_H
