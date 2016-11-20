#include "tansa/jocsPlayer.h"

#include <unistd.h>

namespace tansa {

	double EPSILON = 0.001;

	std::vector<Point> JocsPlayer::getHomes() {
		return homes;
	}

	std::vector<std::vector<Action*>> JocsPlayer::getActions() {
		return actions;
	}

	JocsPlayer::JocsPlayer(const std::vector<Vehicle *> &vehicles, const std::vector<unsigned> &jocsActiveIds) {
		this->vehicles = vehicles;
		this->jocsActiveIds = jocsActiveIds;

		int n = vehicles.size();

		hovers.resize(n);
		for(int i = 0; i < n; i++) {
			hovers[i] = new HoverController(vehicles[i]);
		}

		posctls.resize(n);
		for(int i = 0; i < n; i++) {
			posctls[i] = new PositionController(vehicles[i]);
		}

		lightctls.resize(n);
		for(int i = 0; i < n; i++) {
			lightctls[i] = new LightController(vehicles[i]);
		}

		states.resize(n);
		for(auto& state : states){
			state = StateInit;
		}

		plans.resize(n);
		for(auto &p : plans) {
			p = 0;
		}

		lightCounters.resize(n);
		for(auto &lc : lightCounters) {
			lc = 0;
		}

		transitionStarts.resize(n, Time(0,0));
	}

	/**
	 * Load JOCS data from a specified path
	 */
	void JocsPlayer::loadJocs(Jocs *j) {
		currentJocs = j;
		homes = currentJocs->GetHomes();
		actions = currentJocs->GetActions();
		lightActions = currentJocs->GetLightActions();
		breakpoints = currentJocs->GetBreakpoints();
	}


	/**
	 * Play one 'step' in the choreography
	 */
	void JocsPlayer::step() {

		int n = jocsActiveIds.size();


		// Check for state transitions
		if(states[0] == StateInit) {
			// No implicit transitions
		}
		else if(states[0] == StateArming) {
			// Once all are armed, take them all off to the current position

			bool allGood = true;
			for(int i = 0; i < n; i++) {
				if (vehicles[i]->mode != "offboard" || !vehicles[i]->armed) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				for(auto& state : states) {
					state = StateReady; // TODO: Shouldn't this be StateReady?
				}

				transitions.resize(n);
				for(int i = 0; i < n; i++) {


					states[i] = StateTakeoff;
					transitionStarts[i] = Time::now();
					transitions[i] = new LinearTrajectory(vehicles[i]->state.position, 0, homes[jocsActiveIds[i]], 10.0);
				}
				// TODO: Only grab the ones for the active drones
				holdpoints = homes;

				return;
			}
		}

		// Do the control loops
		for(int i = 0; i < n; i++) {
			Vehicle &v = *vehicles[i];
			auto &s = states[i];

			int chorI = jocsActiveIds[i];


			if(s == StateArming) {
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
				v.setpoint_accel(Vector3d(0,0,0));
			}
			else if(s == StateTakeoff) {
				double t = Time::now().since(transitionStarts[i]).seconds();

				if(transitions[i] == NULL || t >= transitions[i]->endTime()) {
					s = StateHolding;
				}

				posctls[i]->track(transitions[i]);
				posctls[i]->control(t);
			}
			else if(s == StateReady) {
				// Do nothing
				v.setpoint_accel(Vector3d(0,0,0));
			}
			else if(s == StateHolding) {
				if (pauseRequested) {
					paused = true;
					pauseRequested = false;
					pauseOffset = Time::now().seconds();
				}

				double t = Time::now().seconds();
				if (paused && (stopRequested || t - pauseOffset > 20.0)) {
					land();
					return;
				}
				// Do a hover
				hovers[i]->setPoint(holdpoints[i]); // TODO: Only do this on transitions (when holdpoints changes)
				hovers[i]->control(t);
			}
			else if(s == StateFlying) {
				double t = Time::now().since(start).seconds() - pauseOffset;

				Trajectory *motion = static_cast<MotionAction*>(actions[chorI][plans[i]])->GetPath();

				if(t >= actions[chorI][plans[i]]->GetEndTime()) {
					if(plans[i] == actions[chorI].size()-1) {
						states[i] = StateLanding;

						Point lastPoint = motion->evaluate(t).position;
						Point groundPoint = lastPoint; groundPoint.z() = 0;
						transitions[i] = new LinearTrajectory(lastPoint, 0, groundPoint, 10.0);
						transitionStarts[i] = Time::now();
						continue;
					}
					plans[i]++;
				} else if (pauseRequested) {
					double nextBreakpoint = getNextBreakpointTime(t);
					if ((int)t + 1 == nextBreakpoint) {
						states[i] = StateHolding;
						holdpoints[i] = motion->evaluate(t).position;
						continue;
					}
				}

				posctls[i]->track(motion);
				posctls[i]->control(t);


				LightTrajectory *light = static_cast<LightAction *>(lightActions[jocsActiveIds[i]][lightCounters[i]])->GetPath();

				// TODO: this will only work if there is a light action at the same time as a motion action
				// so we should fix that.. it should be possible to have a time where there is only a light action
				// and no motion action.
				if (t >= lightActions[jocsActiveIds[i]][lightCounters[i]]->GetStartTime()) {
					//printf("Getting light action for drone %d at time %f\n", jocsActiveIds[vi], light->getStartTime());
					lightctls[i]->track(light);
					lightctls[i]->control(t);
					if (t >= lightActions[jocsActiveIds[i]][lightCounters[i]]->GetEndTime()) {
						lightCounters[i]++;
					}
				}
			}
			else if(s == StateLanding) {
				double t = Time::now().since(transitionStarts[i]).seconds();

				// Descend to ground
				if(t < transitions[i]->endTime()) {
					posctls[i]->track(transitions[i]);
					posctls[i]->control(t);
				}
				// Disarm
				else if(v.armed){
					v.setpoint_accel(Vector3d(0,0,0));
					v.arm(false);
				}
				// Reset state machine
				else {
					s = StateInit;
				}

			}
		}

		stepTick++;
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

		if (paused) {
			paused = false;
			pauseRequested = false;
			stopRequested = false;
			pauseOffset = 0.0;
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
		// TODO: Determine a pause-at index (and maybe also a stop-at index)
	}

	void JocsPlayer::stop() {
		if (!paused) {
			printf("Must already be paused.");
			return;
		}
		stopRequested = true;
	}

	void JocsPlayer::land() {
		for(auto s : states) {
			if(s != StateHolding) {
				printf("Cannot land: Some drones not holding position\n");
				return;
			}
		}

		for(int i = 0; i < states.size(); i++) {
			states[i] = StateLanding;

			Point lastPoint = holdpoints[i];
			Point groundPoint = lastPoint; groundPoint.z() = 0;
			transitions[i] = new LinearTrajectory(lastPoint, 0, groundPoint, 10.0);
			transitionStarts[i] = Time::now();
		}
	}

	/**
	 * Rewind the chreography by a number of 'steps'
	 * @param steps How far back in the choreography to rewind
	 */
	void JocsPlayer::rewind(int steps) {

	}


	double JocsPlayer::currentTime() {
		if(!this->isPlaying()) {
			return -1;
		}

		return Time::now().since(start).seconds();
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

		for (int i = 0; i < lightctls.size(); i++) {
			delete lightctls[i];
		}

		for (int i = 0; i < hovers.size(); i++) {
			delete hovers[i];
		}

		// TODO: They will memory leak
		for (int i = 0; i < transitions.size(); i++) {
			delete transitions[i];
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
			if (actionStartTime == startTime && isMotionAction(actions[droneId][j])) {
				return ((MotionAction*)actions[droneId][j])->GetStartPoint(); // TODO: check if this cast works...
			}
		}

		return Point(0,0,-1); // TODO: figure out better "invalid" answer
	}
	bool JocsPlayer::isMotionAction(Action* a) {
		MotionAction* ma = dynamic_cast<MotionAction*>(a);
		return ma;
	}
/*
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

	*/
}
