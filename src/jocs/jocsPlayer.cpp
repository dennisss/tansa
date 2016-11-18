#include "tansa/jocsPlayer.h"

#include <unistd.h>

namespace tansa {

	std::vector<Point> JocsPlayer::getHomes() {
		return homes;
	}

	std::vector<std::vector<Action*>> JocsPlayer::getActions() {
		return actions;
	}

	JocsPlayer::JocsPlayer(std::string jocsPath, double scale) {
		this->loadJocs(jocsPath, scale);
	}

	/**
	 * Load JOCS data from a specified path
	 */
	void JocsPlayer::loadJocs(std::string jocsPath, double scale) {
		// TODO: Clear all these if they already have data.
		currentJocs = Jocs::Parse(jocsPath, scale);
		homes = currentJocs->GetHomes();
		actions = currentJocs->GetActions();
	}

	void JocsPlayer::initControllers(std::vector<Vehicle *> vehicles, std::vector<unsigned> jocsActiveIds) {
		// TODO: Have a better check for mocap initialization/health
		sleep(15);

		int n = vehicles.size();

		hovers.resize(n);
		for(int i = 0; i < n; i++) {
			hovers[i] = new HoverController(vehicles[i], homes[jocsActiveIds[i]]);
		}

		posctls.resize(n);
		for(int i = 0; i < n; i++) {
			posctls[i] = new PositionController(vehicles[i]);
		}


		takeoffs.resize(n);
		for(int i = 0; i < n; i++) {
			takeoffs[i] = new LinearTrajectory(vehicles[i]->state.position, 0, homes[jocsActiveIds[i]], 10.0);
		}

		states.resize(n);
		for(auto& state : states){
			state = STATE_INIT;
		}

		plans.resize(n);
		for(auto &p : plans) {
			p = 0;
		}
	}

	/**
	 * Play one 'step' in the choreography
	 */
	void JocsPlayer::step(std::vector<Vehicle *> vehicles, std::vector<unsigned> jocsActiveIds) {

		if(!playing) { // TODO: Playing should be a state achievable only after arming
			return;
		}

		double t = Time::now().since(start).seconds();
		stepTick++;


		int n = vehicles.size();

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
			}

		} else if (states[0] == STATE_TAKEOFF) {
			if (t >= 10.0) {
				for(auto& state : states){
					state = STATE_FLYING;
				}
				start = Time::now();
			}
		}


		// Do the control loops
		for(int vi = 0; vi < n; vi++) {
			Vehicle &v = *vehicles[vi];

			if (states[vi] == STATE_INIT) {
				// Lower frequency state management
				if (stepTick % 50 == 0) {

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
						continue;
					}
					plans[vi]++;
				}
				Trajectory *cur = static_cast<MotionAction*>(actions[jocsActiveIds[vi]][plans[vi]])->GetPath();
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}
		}


		// Final transition
		bool allLanded = true;
		for(int vi = 0; vi < n; vi++) {
			if(states[vi] != STATE_LANDING) {
				allLanded = false;
				break;
			}
		}

		// The chose choreography has ended
		if(allLanded) {
			playing = false;
		}
	}

	void JocsPlayer::play() {
		playing = true;
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
