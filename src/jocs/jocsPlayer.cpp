#include "tansa/jocsPlayer.h"

#include <unistd.h>

namespace tansa {

	std::vector<Point> JocsPlayer::getHomes() {
		return homes;
	}

	std::vector<std::vector<Action*>> JocsPlayer::getActions() {
		return actions;
	}

	JocsPlayer::JocsPlayer(const std::vector<Vehicle *> &vehicles, const std::vector<unsigned> &jocsActiveIds) {
		this->vehicles = vehicles;
		this->jocsActiveIds = jocsActiveIds;
	}

	/**
	 * Load JOCS data from a specified path
	 */
	void JocsPlayer::loadJocs(Jocs *j) {
		// TODO: Clear all these if they already have data.
		currentJocs = j; // Jocs::Parse(jocsPath, scale);
		homes = currentJocs->GetHomes();
		actions = currentJocs->GetActions();
	}

	void JocsPlayer::initControllers() {

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
		// TODO: Only grab the ones for the active drones
		holdpoints = homes;

		states.resize(n);
		for(auto& state : states){
			state = StateInit;
		}

		plans.resize(n);
		for(auto &p : plans) {
			p = 0;
		}
	}


	/**
	 * Play one 'step' in the choreography
	 */
	void JocsPlayer::step() {

		int n = vehicles.size();


		// Check for state transitions
		if(states[0] == StateInit) {
			// No implicit transitions
		}
		else if(states[0] == StateArming) {
			// Once all are armed, take them all off to the current position

			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if (vehicles[vi]->mode != "offboard" || !vehicles[vi]->armed) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				for(auto& state : states) {
					state = StateTakeoff; // TODO: Shouldn't this be StateReady?
				}

				return;
			}
		}

		// TODO:
		// If all are Ready, then transition to takeoff



		double t = Time::now().since(start).seconds();
		stepTick++;


		// If any are taking off, track when they are done
		for(int i = 0; i < n; i++) {
			if(states[i] == StateTakeoff) {
				if(takeoffs[i] == NULL || t >= takeoffs[i]->endTime()) {
					states[i] = StateHolding;
				}
			}
		}




		// Do the control loops
		for(int vi = 0; vi < n; vi++) {
			Vehicle &v = *vehicles[vi];


			if(states[vi] == StateArming) {
				// Lower frequency state management
				if(stepTick % 50 == 0) {
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
			}
			else if(states[vi] == StateTakeoff) {
				posctls[vi]->track(takeoffs[vi]);
				posctls[vi]->control(t);
			}
			else if(states[vi] == StateReady) {
				// Do nothing
				vehicles[vi]->setpoint_accel(Vector3d(0,0,0));
			}
			else if(states[vi] == StateHolding) {
				// Do a hover
				hovers[vi]->setPoint(holdpoints[vi]); // TODO: Only do this on transitions (when holdpoints changes)
				hovers[vi]->control(t);
			}
			else if(states[vi] == StateFlying) {
				if(t >= actions[jocsActiveIds[vi]][plans[vi]]->GetEndTime()) {
					if(plans[vi] == actions[jocsActiveIds[vi]].size()-1) {
						states[vi] = StateLanding;
						v.land();
						continue;
					}
					plans[vi]++;
				}
				Trajectory *cur = static_cast<MotionAction*>(actions[jocsActiveIds[vi]][plans[vi]])->GetPath();
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}
			else if(states[vi] == StateLanding) {


				// TODO: Do a trajectory down to 0
				// Once done, do nothing and disarm the drone

				// Transition back to StateInit

			}
		}
	}

	void JocsPlayer::prepare() {

		for(auto s : states) {
			if(s != StateInit) {
				printf("Cannot prepare: Some drones not in initial state\n");
				return;
			}
		}


		for(auto &s : states) {
			s = StateArming;
		}
	}

	void JocsPlayer::play() {

		for(auto s : states) {
			if(s != StateHolding) {
				printf("Cannot play: Some drones not ready\n");
				return;
			}
		}

		start = Time::now();
		for(auto &s : states) {
			s = StateFlying;
		}
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

	void JocsPlayer::createBreakpointSection(const std::vector<vector> actions, int startPoint, int endPoint = -1) {
		double startTime, endTime = -1;
		int bpStartIndex, bpEndIndex = -1;
		if(startPoint >= 0) {
			startTime = getBreakpointTime(startPoint);
		} else {
			bpStartIndex = 0;
		}
		if(startPoint >= 0) {
			endTime = getBreakpointTime(endPoint);
		} else {
			bpEndIndex = actions.size() - 1;
		}
		for(int i = 0; i < actions.size(); i++) {
			if(actions[i][1].start_time.seconds == startTime) {
				bpStartIndex = i;
			}
			if(actions[i][1].start_time.seconds == endTime) {
				bpEndIndex = i;
			}
			if(bpStartIndex != -1 && bpEndIndex != -1) {
				break;
			}
		}
		startIndex = bpStartIndex;
		endIndex = bpEndIndex;
	}
}
