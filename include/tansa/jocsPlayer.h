#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include <tansa/core.h>
#include <tansa/control.h>
#include <tansa/jocsParser.h>
#include <tansa/config.h>

#include "tansa/routine.h"
#include "tansa/config.h"
#include <fstream>


namespace tansa {
/**
 * A controller class for executing a choreography.
 */
class JocsPlayer {
public:
	JocsPlayer(bool inRealLife) { this->inRealLife = inRealLife; }


	/**
	 * Initialize all vehicles, required for flying
	 * @param vehicles The list of vehicles in the choreography that should be initialized.
	 */
	void initVehicles(const std::vector<Vehicle *> &vehicles);
	/**
	 * 	Get ready to fly (arm and takeoff to home point)
	 * 	Does not start the choreography. Only moves all drones to starting points.
	 */
	void prepare();
	/**
	 * Actually start flying the choreography.
	 */
	void play();
	/**
	 * Pause the choreography at the next breakpoint. Only supported at breakpoints. Cannot pause while drones are moving.
	 */
	void pause();
	/**
	 * Land all the drones that are currently flying.
	 */
	void land();
	/**
	 * Must be already paused for this to work. Stops the choreography and lands all the drones safely.
	 */
	void stop();
	/**
	 * Proceed one time step in the choreography.
	 */
	void step();

	/**
	 * Causes all vehicles to enter a failsafe landing mode
	 */
	void failsafe();


	/**
	 * Reorder the drones to their closest tracks 
	 */
	void rearrange();

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
	 * @return Are we in the middle of a choreography?
	 */
	inline bool isPlaying() { return states.size() > 0 && this->states[0] == StateFlying; }
	/**
	 * @return Are we ready to start the choreography ("Play")
	 */
	inline bool isReady() { return states.size() > 0 && this->states[0] == StateHolding; }
	/**
	 * @return Are the drones currently in a paused state?
	 */
	inline bool isPaused() { return this->paused; }
	/**
	 * @return Have the drones landed?
	 */
	inline bool isLanded() { return this->landed; }
	/**
	 * Gets the time relative to the start of the current file
	 */
	double currentTime();
	/**
	 * @return The homes positions of the drones in the currently loaded jocs file.
	 */
	std::vector<Point> getHomes();
	/**
	 * @return The actions of the currently loaded jocs file.
	 */
	std::vector<std::vector<Action*>> getActions();
	/**
	 * @return The breakpoints of the currently loaded jocs file.
	 */
	std::vector<Breakpoint> getBreakpoints();

	/**
	 * @return list of track/role numbers which will be used
	 */
	inline std::vector<unsigned> getActiveTracks() { return jocsActiveIds; }

	/**
	 * Cleanup any resources laying around. Only call after done flying and during shutdown process.
	 */
	void cleanup();


	inline std::vector<PlayerVehicleState> getStates() { return this->states; }

	Routine *getCurrentFile() { return currentJocs; }

private:
	std::vector<Vehicle *> vehicles;
	std::vector<vehicle_config> vehicleConfigs;
	std::vector<unsigned> jocsActiveIds;

	std::vector<Breakpoint> breakpoints;
	Routine *currentJocs = nullptr;
	std::vector<std::vector<Action*>> actions;
	//3d vector. 1 track for each drone, then each drone has LightController::NUM_LIGHTS
	//tracks for actions. Actions applied per light per drone so they can be controlled
	//separately
	std::vector<std::vector<std::vector<LightAction*>>> lightActions;
	std::vector<Point> homes;
	std::vector<HoverController *> hovers;
	std::vector<PositionController *> posctls;
	std::vector<LightController *> lightctls;
	std::vector<std::vector<int>> lightCounters;
	std::vector<Point> holdpoints;
	std::vector<PlayerVehicleState> states;
	std::vector<int> plans;

	// Separate trajectories and timings for doing takeoff and landings
	std::vector<Trajectory::Ptr> transitions;
	vector<Time> transitionStarts;
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
	bool looping = false;
	bool inRealLife;

	ofstream logfile;

	void reset();
	double getNextBreakpointTime(double lastTime);
	double getBreakpointTime(unsigned breakpointNumber);
	double getBreakpointTime(std::string breakpointName);
	unsigned getBreakpointNumber(double startTime);
	Point getDroneLocationAtTime(double startTime, unsigned droneId);
	bool isMotionAction(Action* a);
	void createBreakpointSection(int startPoint, int endPoint = -1);
	void log();
};

}
#endif //TANSA_JOCSPLAYER_H
