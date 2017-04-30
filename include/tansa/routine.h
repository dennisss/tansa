#ifndef TANSA_ROUTINE_H
#define TANSA_ROUTINE_H

#include "action.h"
#include "breakpoint.h"
#include "manager.h"

// TODO: Eventually get rid of this header
#include "config.h"

#include <memory>
#include <string>
#include <vector>

namespace tansa {


/**
 * A generic object containing actions that should be executed by vehicles
 */
class Routine {
public:

	virtual ~Routine() {}

	typedef std::shared_ptr<Routine> Ptr;

	// TODO: Maybe decouple the scale?
	static Routine *Load(std::string path, double scale = 1.0);

	/**
	 * Determines if some path to a file has the extension of a valid routing type
	 */
	static bool IsFile(std::string path);


	double duration();

	std::vector<std::vector<Action*>> actions;
	std::vector<Breakpoint> breakpoints;
	std::vector<Point> homes;
	std::vector<std::vector<std::vector<LightAction*>>> lightActions;
	bool needConvertToMeters = false;
	bool needConvertToRadians = false;


};


class RoutinePlayer : ManagerProgram {

public:


	virtual void step(double t);


	/**
	 * Should be called to load a jocs file.
	 * @param jocsPath The path to the jocs file.
	 * @param scale Default should be 1.0f Applies a uniform scaling to all lengths in the choreography.
	 * @param jocsActiveIds Which drones are actually being used? Non active drones don't fly.
	 * @param start Which breakpoint should we start at? Defaults to -1 which means start from the beginning.
	 * @return whether or not the load operation suceeded
	 */
	bool loadChoreography(string jocsPath, float scale, const std::vector<unsigned> &jocsActiveIds, int start = -1);
	bool loadChoreography(Routine *chor, const std::vector<unsigned> &jocsActiveIds, int start = -1);
	// TODO: Instead we should use isRunning which checks if any states are not StateInit
	bool canLoad();

	/**
	 * @return The actions of the currently loaded jocs file.
	 */
	std::vector<std::vector<Action*>> getActions();
	/**
	 * @return The breakpoints of the currently loaded jocs file.
	 */
	std::vector<Breakpoint> getBreakpoints();


	/**
	 * Cleanup any resources laying around. Only call after done flying and during shutdown process.
	 */
	void cleanup();



	Routine *getCurrentFile() { return currentRoutine; }

private:
	std::vector<Vehicle::Ptr> vehicles;
	std::vector<vehicle_config> vehicleConfigs;
	std::vector<unsigned> jocsActiveIds;

private:


	std::vector<Breakpoint> breakpoints;
	Routine *currentRoutine = nullptr;
	std::vector<std::vector<Action*>> actions;
	//3d vector. 1 track for each drone, then each drone has LightController::NUM_LIGHTS
	//tracks for actions. Actions applied per light per drone so they can be controlled
	//separately
	std::vector<std::vector<std::vector<LightAction*>>> lightActions;
	std::vector<Point> homes;
	std::vector<std::vector<int>> lightCounters;
	std::vector<Point> holdpoints;

	std::vector<int> plans;

	bool pauseRequested = false;
	bool paused = false;
	bool stopRequested = false;
	bool landed = false;
	Time start = Time(0,0); // TODO: I will also need a time offset
	Time pauseOffset = Time(0,0);
	double timeOffset = 0.0;
	std::vector<int> pauseIndices;
	int stepTick = 0;
	std::vector<int> startIndices;
	std::vector<int> endIndices;
	double startOffset = 0.0;


	void reset();
	double getNextBreakpointTime(double lastTime);
	double getBreakpointTime(unsigned breakpointNumber);
	double getBreakpointTime(std::string breakpointName);
	unsigned getBreakpointNumber(double startTime);
	Point getDroneLocationAtTime(double startTime, unsigned droneId);
	bool isMotionAction(Action* a);
	void createBreakpointSection(int startPoint, int endPoint = -1);

};



/**
 * Used to verify the correctness of a Routine by determining if it is physically possible
 */
class FeasibilityChecker {
public:
	FeasibilityChecker() {};


	bool check(Routine &r);
	bool check(Trajectory::Ptr traj);

	void reset() { errors.resize(0); }

	std::vector<std::string> errors;


};


}


#endif
