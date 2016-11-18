#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <zconf.h>
#include <tansa/control.h>
#include "tansa/jocsPlayer.h"

namespace tansa {
	const unsigned JocsPlayer::STATE_INIT = 0;
	const unsigned JocsPlayer::STATE_TAKEOFF = 0;
	const unsigned JocsPlayer::STATE_FLYING = 0;
	const unsigned JocsPlayer::STATE_LANDING = 0;

	bool JocsPlayer::isInitialized() {
		return initialized;
	}

	JocsPlayer::JocsPlayer(bool withMocap, hardware_config config, string jocsPath, std::vector<unsigned> jocsActiveIds, double scale) {
		this->useMocap = withMocap;
		this->loadJocs(jocsPath, jocsActiveIds, scale);
		n = spawns.size();

		// Only pay attention to homes of active drones
		if (useMocap) {
			mocap = new Mocap();
			mocap->connect(config.clientAddress, config.serverAddress);
		} else {
			gazebo = new GazeboConnector();
			gazebo->connect();
			gazebo->spawn(spawns);
		}
	}

	/**
	 * Load JOCS data from a specified path
	 */
	void JocsPlayer::loadJocs(string jocsPath, std::vector<unsigned> jocsActiveIds, double scale) {
		// TODO: Clear all these if they already have data.
		auto jocsData = Jocs::Parse(jocsPath, scale);
		homes = jocsData.GetHomes();
		actions = jocsData.GetActions();

		for (int i = 0; i < jocsActiveIds.size(); i++) {
			int chosenId = jocsActiveIds[i];
			// We assume the user only configured for valid IDs..
			spawns.push_back(homes[chosenId]);
			spawns[i].z() = 0;
		}
	}

	void JocsPlayer::initVehicles(std::vector<vehicle_config> vconfigs, std::vector<unsigned> jocsActiveIds) {
		if (n > vconfigs.size()) {
			printf("Not enough drones on the network\n");
			return;
		}

		vehicles.reserve(n);

		for(int i = 0; i < n; i++) {
			const vehicle_config &v = vconfigs[i];

			vehicles[i] = new Vehicle();
			vehicles[i]->connect(v.lport, v.rport);
			if (useMocap) {
				mocap->track(vehicles[i], i+1);
			} else {
				gazebo->track(vehicles[i], i);
			}
		}

		// TODO: Have a better check for mocap initialization/health
		sleep(15);

		hovers.reserve(n);
		for(int i = 0; i < n; i++) {
			hovers[i] = new HoverController(vehicles[i], homes[jocsActiveIds[i]]);
		}

		posctls.reserve(n);
		for(int i = 0; i < n; i++) {
			posctls[i] = new PositionController(vehicles[i]);
		}


		takeoffs.reserve(n);
		for(int i = 0; i < n; i++) {
			takeoffs[i] = new LinearTrajectory(vehicles[i]->state.position, 0, homes[jocsActiveIds[i]], 10.0);
		}

		states.resize(n);
		for(auto& state : states){
			state = STATE_INIT;
		}
		initialized = true;
		// Index of the current action running for each drone (initially running the 0th)
		plans.resize(n);
	}

	/**
	 * Play one 'step' in the choreography
	 */
	Time JocsPlayer::play(Time start, int i, int &numLanded, bool &running, std::vector<unsigned> jocsActiveIds) {
		double t = Time::now().since(start).seconds();

		// Check for state transitions
		if (states[0] == STATE_INIT) {

			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if (vehicles[vi]->mode != "offboard" || !vehicles[vi]->armed) {
					allGood = false;
					break;
				}
			}

			if (allGood) {
				start = Time::now();
				for(auto& state :states) {
					state = STATE_TAKEOFF;
				}
				return start;
			}

		} else if (states[0] == STATE_TAKEOFF) {
			if (t >= 10.0) {
				for(auto& state : states){
					state = STATE_FLYING;
				}
				start = Time::now();
				return start;
			}
		}


		// Do the control loops
		for(int vi = 0; vi < n; vi++) {
			Vehicle &v = *vehicles[vi];

			if (states[vi] == STATE_INIT) {
				// Lower frequency state management
				if (i % 50 == 0) {

					if (v.mode != "offboard") {
						v.set_mode("offboard");
						printf("Setting mode\n");
					} else if (!v.armed) {
						v.arm(true);
						printf("Arming mode\n");
					}
				}


				// Do nothing
				vehicles[vi]->setpoint_accel(Vector3d(0,0,0));
			} else if (states[vi] == STATE_TAKEOFF) {
				posctls[vi]->track(takeoffs[vi]);
				posctls[vi]->control(t);
			} else if (states[vi] == STATE_FLYING) {
				if (t >= actions[jocsActiveIds[vi]][plans[vi]]->GetEndTime()) {
					if (plans[vi] == actions[jocsActiveIds[vi]].size()-1) {
						states[vi] = STATE_LANDING;
						v.land();
						numLanded++;
						if (numLanded == n) {
							running = false;
						}
						continue;
					}
					plans[vi]++;
				}
				Trajectory *cur = static_cast<MotionAction*>(actions[jocsActiveIds[vi]][plans[vi]])->GetPath();
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}
		}
		return start;
	}

	/**
	 * Pause the choreography
	 */
	void JocsPlayer::pause() {
		printf("Pause requested!");
		pauseRequested = true;
	}

	/**
	 * Rewind the chreography by a number of 'steps'
	 * @param steps How far back in the choreography to rewind
	 */
	void JocsPlayer::rewind(int steps) {

	}

	/**
	 * Reset the choreography back to the initial position (ie: plans[0])
	 */
	void JocsPlayer::reset() {
		resetMode = true;
	}


	void JocsPlayer::cleanup() {
		if (useMocap) {
			mocap->disconnect();
			delete JocsPlayer::mocap;
		} else {
			gazebo->disconnect();
			delete gazebo;
		}

		// Stop all vehicles
		for (int vi = 0; vi < n; vi++) {
			Vehicle *v = vehicles[vi];
			v->disconnect();
			delete v;
		}

		for (int i = 0; i < posctls.size(); i++) {
			delete posctls[i];
		}

		for (int i = 0; i < hovers.size(); i++) {
			delete hovers[i];
		}

		for (int i = 0; i < takeoffs.size(); i++) {
			delete takeoffs[i];
		}
	}

	/**
     * Helper methods
     * TODO: write test cases (especially since the code doesn't use them yet)
     */

	double JocsPlayer::getNextBreakpointTime(double lastTime) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		// Assumes list of breakpoints is in time-ascending order
		for (unsigned i = 0; i < breakpointsLength - 1; i++) {
			double ret = breakpoints[i].GetStartTime();
			if (ret >= lastTime) {
				// Return the first breakpoint whose starttime hasn't passed yet
				// or is currently happening
				return ret;
			}
		}

		return -1;
	}
	double JocsPlayer::getBreakpointTime(unsigned breakpointNumber) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		for (unsigned i = 1; i < breakpointsLength - 1; i++) {
			unsigned ret = breakpoints[i].GetNumber();
			if (ret == breakpointNumber) {
				return breakpoints[i].GetStartTime();
			}
		}

		return -1;
	}
	double JocsPlayer::getBreakpointTime(std::string breakpointName) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		for (unsigned i = 1; i < breakpointsLength - 1; i++) {
			std::string name = breakpoints[i].GetName();
			if (name.compare(breakpointName) == 0) {
				return breakpoints[i].GetStartTime();
			}
		}

		return -1;
	}
	unsigned JocsPlayer::getBreakpointNumber(double startTime) {
		unsigned breakpointsLength = breakpoints.size();

		// Cycles all breakpoints
		for (unsigned i = 1; i < breakpointsLength - 1; i++) {
			double ret = breakpoints[i].GetStartTime();
			if (ret > startTime) {
				// Return the breakpoint that the given time is a part of
				// Also works for given times that are within a breakpoint
				// not just the start time of a breakpoint
				return breakpoints[i-1].GetNumber();
			}
		}

		return -1;
	}
	Point JocsPlayer::getDroneLocationAtTime(double startTime, unsigned droneId) {
		// This one only works if the startTime is a startTime of an action
		// Returns negative value if startTime is either between actions or out of scope

		unsigned actionsLength = actions[droneId].size();

		// Assume droneId is valid and use it to choose which list of actions we need to parse
		for (unsigned j = 0; j < actionsLength; j++) {
			double actionStartTime = actions[droneId][j]->GetStartTime();
			bool isMotionAction = true; // TODO: actually check if it's a motion. For now, they're all motions.
			if (actionStartTime == startTime && isMotionAction) {
				return ((MotionAction*)actions[droneId][j])->GetStartPoint(); // TODO: check if this cast works...
			}
		}

		return Point(0,0,-1); // TODO: figure out better "invalid" answer
	}
}
