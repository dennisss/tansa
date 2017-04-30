#ifndef TANSA_MANAGER_H_
#define TANSA_MANAGER_H_

#include "core.h"
#include "vehicle.h"
#include "control.h"

// TODO: Make these configurable
#define VEHICLE_ASCENT_MS 0.2
#define VEHICLE_DESCENT_MS 0.3

namespace tansa {

/*
	1. The manager will be given a list of vehicles (or several pools)
	2. A program is loaded (it will provide the desired start positions of some number of drones)
	3. Depending on the options, vehicles will be automatically assigned to the program
		- Or, the mapping between vehicle ids and program roles can be user specified
	4.

*/

/**
 * State assigned to a vehicle controlled by a player
 */
enum ManagerVehicleState {
	/**
	 * Vehicle disarmed and not doing anything
	 *
	 * Transitions to Ready upon arm()
	 */
	StateInit,

	/**
	 * Arms the drone, transitions to Ready on completion
	 */
	StateArming,

	/**
	 * Vehicle armed but doing nothing on the ground
	 *
	 * Once all vehicles are Ready, transitions to Takeoff on load of a JOCS file
	 */
	StateReady,

	/**
	 * Vehicle going to starting point of first action
	 *
	 * If the starting point is on the ground, then:
	 * - if the vehicle is on the ground, this is a no-op
	 * - else this descends and lands without entering Landing state
	 *
	 * Transitions to Holding upon getting to the point
	 */
	StateTakeoff,

	/**
	 * Hovers/sits in one place
	 *
	 * If in the air and 20 seconds have elapsed, transitions to Landing state
	 *
	 * Transitions to Takeoff upon load of a new JOCS file
	 *
	 * Transitions to Flying on play()
	 */
	StateHolding,

	/**
	 * Performs the assigned actions
	 *
	 * After a pause(), on next safe point, transitions to Holding
	 *
	 * Transitions to Landing after completion of all actions
	 */
	StateFlying,

	/**
	 * Landing at the current position and disarming
	 *
	 * Transitions to Init upon completion
	 */
	StateLanding,

	/**
	 * Vehicle responding to a failure condition
	 */
	StateFailsafe
};



/**
 * Interface for making programs that can
 */
class ManagerProgram {
public:



	// Some functions to define the preconditions for this program to run
	std::vector<Vector3d> get_start_locations();


	virtual void step(double t) = 0;

};



/**
 * A state machine for coordinating the lifecycles of multiple drones and allow separate programs to run
 *
 * The general workflow is to:
 * 1. Make a Manager
 * 2. Add vehicles to it
 * 3. Load a program
 * 4. Assign vehicles to the program
 * 5. Prepare the vehicles into the states needed for running the program
 * 6. Run the program
 * 7. Repeat from 3 as needed
 */
class Manager {
public:
	Manager(bool inRealLife) { this->inRealLife = inRealLife; }



	void add_vehicles(VehiclePool::Ptr vehicles);



	/**
	 * Initialize all vehicles, required for flying
	 * @param vehicles The list of vehicles in the choreography that should be initialized.
	 */
	void initVehicles(const std::vector<Vehicle::Ptr> &vehicles);
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
	 * Causes all vehicles to enter a failsafe landing mode
	 */
	void failsafe();


	/**
	 * Reorder the drones to their closest tracks
	 */
	void rearrange();



	/**
	 * Should be called at 100Hz to update the drones
	 */
	void step();

	// Some ability to specify roles in this




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
	//inline bool isLanded() { return this->landed; }

	/**
	 * Gets the time relative to the start of the current file
	 */
	double currentTime();


	inline std::vector<ManagerVehicleState> getStates() { return this->states; }


	/**
	 * @return The homes positions of the drones in the currently loaded jocs file.
	 */
	std::vector<Point> getHomes();

	/**
	 * @return list of track/role numbers which will be used
	 */
	inline std::vector<unsigned> getActiveTracks() { return activeIds; }



	// Used by programs and the manager to control the drones
	std::vector<HoverController *> hovers;
	std::vector<PositionController *> posctls;
	std::vector<LightController *> lightctls;

private:

	void log();



	ManagerProgram *program;
	std::vector<Vehicle::Ptr> vehicles; /**< The  */
	std::vector<unsigned> activeIds; /**< TODO: Something to maintain the current assignment of vehicles to roles. TODO: Why not just reorder the vehicles array? <- Except if we reorder the array, then we need to account for that by also reordering the states, transitions, etc.  */

	std::vector<PlayerVehicleState> states;

	// Separate trajectories and timings for doing takeoff and landings
	std::vector<Trajectory::Ptr> transitions;
	vector<Time> transitionStarts;


	bool looping = false;
	bool inRealLife;

	ofstream logfile;


};






};


#endif
