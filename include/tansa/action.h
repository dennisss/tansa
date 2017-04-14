#ifndef TANSA_ACTION_H
#define TANSA_ACTION_H

#include "trajectory.h"
#include "control.h"
#include <memory>

namespace tansa {

/**
 * Type representing the different actions in jocs
 * @enum
 */
enum class ActionTypes : unsigned{
	None 			= 0,
	Transition 		= 1,
	Line 			= 2,
	Circle			= 3,
	Hover 			= 4,
	Light 			= 5,
	Strobe 			= 6,
	Ellipse 		= 7,
	Spiral 			= 8,
	Arc 			= 9,
	TransformedTraj = 10,
	GradualCircle 	= 11,
	Helix           = 12
};

typedef unsigned DroneId;
/**
 * @interface Defines an interface for all Actions to implement
 */
class Action {
public:
	/**
	 * @param id Id of the drone this action refers to
	 * @param type The type of action that this object refers to.
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
	/**
	 * Used polymorphically to determine type of action.
	 * @return what type of action this is.
	 */
	inline ActionTypes GetActionType() { return type; }

	inline bool is_light_action() {
		return type == ActionTypes::Strobe || type == ActionTypes::Light;
	}


	/**
	 * Line number, if applicable, (or -1) from the original file.
	 * This is used to annotate the routines messages
	 */
	int line = -1;
protected:
	DroneId droneId;
	ActionTypes type;
	bool isCalculated = false;
};
/**
 * Represents an action to be replaced by another action (usually for transitions)
 */
class EmptyAction : public Action {
public:
    EmptyAction(DroneId id, double s, double e): Action(id,ActionTypes::Transition), startTime(s), endTime(e){}
	virtual ~EmptyAction(){}
	/**
	 * @virtual
	 * @return Double representing the seconds after initialization that this action should start at
	 */
    virtual double GetStartTime() const { return startTime; }
	/**
	 * @virtual
	 * @return Double representing the seconds after initialization that this action should end at
	 */
    virtual double GetEndTime() const { return endTime; }
private:
    double startTime;
    double endTime;
};
/**
 * Represents an action with a trajectory. Trajectories can be of different types and result in different
 * types of motion actions
 */
class MotionAction : public Action {
public:
	/**
	 * Construct an instance of a action the involves drone movement. Motion actions are always calculated when constructed.
	 * @param id Id of the drone that this action applies to.
	 * @param t The mathematical representation of the path that this action represents.
	 * @param type Type of motion action that this is.
	 * @return Motion action with given parameters
	 */
	MotionAction(DroneId id, Trajectory::Ptr t, ActionTypes type) : Action(id, type), path(t) { isCalculated = true; }
	virtual ~MotionAction(){ }
	/**
	 * Get TrajectoryState of evaluating the path at a given time.
	 * @param t The time in seconds that the path should be evaluated at.
	 * @return The Trajectory state on the given path at the given time.
	 */
	inline TrajectoryState GetPathState(double t){ return path->evaluate(t); }
	/**
	 * @return A pointer to the trajectory object contained within this object.
	 */
	inline Trajectory::Ptr GetPath(){return path;}
	/**
	 * @return The start time in seconds of this object's path.
	 */
	virtual double GetStartTime() const { return path->startTime(); }
	/**
	 * @return The end time in seconds of this object's path
	 */
	virtual double GetEndTime() const { return path->endTime(); }
	/**
	 * @return The start point of the trajectory obtained by evaluating the trajectory at its start time.
	 */
	inline Point GetStartPoint() const { return path->evaluate(path->startTime()).position; }
	/**
	 * @return The end point of the trajectory obtained by evaluating the trajectory at its end time.
	 */
	inline Point GetEndPoint() const { return path->evaluate(path->endTime()).position; }

private:
	Trajectory::Ptr path;
};

/**
 * Represents a light action (turning light on or off, patterns, etc)
 */
class LightAction : public Action {
public:
	/**
	 * Constructs a light action. Takes ownership of the LightTrajectory passed in.
	 * @param did The id of the drone that will execute this action
	 * @param t The trajectory that the light should follow. In other words, how does the light change over time?
	 * @return A LightAction instance that encapsulates the relevant trajectory and state.
	 */
	LightAction(DroneId did, LightTrajectory::Ptr t, LightController::LightIndices light_index) :
			Action(did, ActionTypes::Light), path(t), index(light_index)
			{ isCalculated = true; }
	/**
	 * Deletes its path on deletion. This object owns the trajectory.
	 * TODO: Make this an owned pointer.
	 */
	virtual ~LightAction(){}

	/**
	 * Get intensity value of evaluating the path at a given time.
	 * @param t The time in seconds that the path should be evaluated at.
	 * @return The intensity of the light on the given trajectory at the given time.
	 */
	inline double GetPathState(double t) { return path->evaluate(t); }
	/**
	 * @return The lights trajectory, how it changes over time.
	 */
	inline LightTrajectory::Ptr GetPath() { return path; }
	/**
	 * @return The start time for the trajectory
	 */
	virtual double GetStartTime() const { return path->getStartTime(); }
	/**
	 * @return The end time for the trajectory
	 */
	virtual double GetEndTime() const { return path->getEndTime(); }
	/**
	 * @return The intensity of the light at the start of its trajectory
	 */
	inline double GetStartIntensity() { return path->evaluate(path->getStartIntensity()); }
	/**
	 * @return The intensity of the light at the end of its trajectory
	 */
	inline double GetEndIntensity() { return path->evaluate(path->getEndIntensity()); }
	/**
	 * @return Which light this action is referring to.
	 */
	inline LightController::LightIndices GetLightIndex() { return index; }

private:
	LightController::LightIndices index;
	LightTrajectory::Ptr path;
	//LightId lightId;
};

}

#endif //TANSA_ACTION_H
