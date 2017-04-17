#include "tansa/jocsPlayer.h"

#include <unistd.h>

namespace tansa {

	std::vector<Breakpoint> JocsPlayer::getBreakpoints() {
		return breakpoints;
	}

	std::vector<Point> JocsPlayer::getHomes() {
		return homes;
	}

	std::vector<std::vector<Action*>> JocsPlayer::getActions() {
		return actions;
	}

	void JocsPlayer::initVehicles(const std::vector<Vehicle *> &vehicles) {
		this->vehicles = vehicles;

		int n = vehicles.size();

		hovers.resize(n);
		for(int i = 0; i < n; i++) {
			hovers[i] = new HoverController(vehicles[i]);
		}

		posctls.resize(n);
		for(int i = 0; i < n; i++) {
			posctls[i] = new PositionController(vehicles[i]);
		}

		lightCounters.resize(n);
		for(auto &light : lightCounters) {
			light.resize(LightController::NUM_LIGHTS, 0);
		}

		lightctls.resize(n);
		for(int i = 0; i < n; i++) {
			lightctls[i] = new LightController(vehicles[i]);
		}

		states.resize(n);
		for(auto& state : states){
			state = StateInit;
		}

		transitionStarts.resize(n, Time(0,0));
	}

void JocsPlayer::reset() {
	int n = vehicles.size();

	plans.resize(n);
	for (int i = 0; i < n; i++) {
		plans[i] = startIndices[i];
	}

	lightCounters.resize(n);
	for(auto &light : lightCounters) {
		light.resize(LightController::NUM_LIGHTS, 0);
	}

	pauseIndices.resize(n);
	for(auto &pi : pauseIndices){
		pi = 0;
	}
}


	bool JocsPlayer::canLoad() {
		for (auto s : states) {
			if(s != StateInit) {
				printf("Cannot load a new file: Some drones not in initial state\n");
				return false;
			}
		}

		if (isPlaying()) {
			printf("Can't load a new jocs file if still playing.\n");
			return false;
		}
		return true;
	}

bool JocsPlayer::loadChoreography(string jocsPath, float scale, const std::vector<unsigned> &jocsActiveIds, int start){
	cout << "loadJocs(" << jocsPath << ", " << scale << ", " << jocsActiveIds.size() << ", " << start << ")" << endl;
	Routine *choreography = Routine::Load(jocsPath, scale);

	if(!choreography) {
		cout << "Invalid file!" << endl;
		return false;
	}

	return this->loadChoreography(choreography, jocsActiveIds, start);
}

/**
* Load JOCS data from a specified path
*/
bool JocsPlayer::loadChoreography(Routine *chor, const std::vector<unsigned> &jocsActiveIds, int start) {
		if (!this->canLoad()) {
			return false;
		}

		delete currentJocs;
		this->jocsActiveIds = jocsActiveIds;
		landed = false;

		currentJocs = chor;
		homes = currentJocs->homes;
		actions = currentJocs->actions;
		lightActions = currentJocs->lightActions;
		breakpoints = currentJocs->breakpoints;

		if(homes.size() < jocsActiveIds.size()) {
			this->jocsActiveIds.resize(homes.size());
		}

		if (start > 0) {
			this->createBreakpointSection(start);
			for (int i = 0; i < homes.size(); i++) {
				homes[i] = getDroneLocationAtTime(startOffset, i);
			}
		} else {
			startIndices.resize(homes.size(), 0);
			endIndices.resize(homes.size(), (int)actions.size() - 1);
			startOffset = 0.0;
		}

		cout << "Finished loading jocs" << endl;

		return true;
	}


	/**
	 * Play one 'step' in the choreography
	 */
	void JocsPlayer::step() {

		int n = jocsActiveIds.size();

		// Check for state transitions
		if (states[0] == StateInit) {
			// No implicit transitions
		} else if (states[0] == StateArming) {
			// Once all are armed, take them all off to the current position

			bool allGood = true;
			for(int i = 0; i < n; i++) {
				if (vehicles[i]->mode != "offboard" || !vehicles[i]->armed) {
					allGood = false;
					break;
				}
			}

			if (allGood) {
				start = Time::now();
				for(auto& state : states) {
					state = StateReady; // TODO: Shouldn't this be StateReady?
				}

				transitions.resize(n);
				for(int i = 0; i < n; i++) {
					Vector3d startPosition = vehicles[i]->state.position,
							 endPosition = homes[jocsActiveIds[i]];

					double dur = (startPosition - endPosition).norm() / VEHICLE_ASCENT_MS;
					states[i] = StateTakeoff;
					transitionStarts[i] = start;
					transitions[i] = Trajectory::Ptr( new LinearTrajectory(startPosition, 0, endPosition, dur) );
				}
				// TODO: Only grab the ones for the active drones
				holdpoints = homes;

				// Opening file for logging data
				logfile.open("log/" + Time::realNow().dateString() + ".csv", ofstream::out);

				return;
			}
		}

		// Do the control loops
		for(int i = 0; i < n; i++) {
			Vehicle &v = *vehicles[i];
			auto &s = states[i];

			int chorI = jocsActiveIds[i];

			// Failsafe, land on motion capture lose
			if(s != StateInit) {
				if(!v.connected) {
					// Flip out to the user!
					cout << "UNCONTROLLABLE: VEHICLE NOT CONNECTED: THIS IS BAD!!" << endl;
				}
				else if(!v.tracking && s != StateFailsafe) {
					cout << "Not tracking, entering failsafe mode" << endl;
					s = StateFailsafe;
				}
			}


			if (s == StateArming) {
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
				v.setpoint_zero();
			} else if (s == StateTakeoff) {
				double t = Time::now().since(transitionStarts[i]).seconds();

				// If the drones started on the ground and overshot the target, switch to hover
				bool overshot = v.state.position.z() > holdpoints[chorI].z();

				if(transitions[i] == NULL || t >= transitions[i]->endTime() || overshot) {
					s = StateHolding;
				}

				posctls[i]->track(transitions[i]);
				posctls[i]->control(t);
			} else if (s == StateReady) {
				// Do nothing
				v.setpoint_zero();
			} else if (s == StateHolding) {
				double t = Time::now().seconds();

				if (pauseRequested) {
					cout << "Transitioning to paused, t = " << Time::now().since(start).seconds() - timeOffset << endl;
					paused = true;
					pauseRequested = false;
					pauseOffset = Time::now();
				}

				if (paused && (stopRequested || Time::now().since(pauseOffset).seconds() > 20.0)) {
					string message = stopRequested ? "Stop requested" : "Paused for more than 20 seconds";
					cout << message + ", attempting to land." << endl;
					paused = false;
					stopRequested = false;
					this->land();
					return;
				}
				// Do a hover
				hovers[i]->setPoint(holdpoints[chorI]); // TODO: Only do this on transitions (when holdpoints changes)
				hovers[i]->control(t);
			} else if (s == StateFlying) {
				this->log();

				double t = Time::now().since(start).seconds() - timeOffset + startOffset;
				if (((int)t) % 5 == 0 && abs(t - (int)t) < 0.01) {
					printf("%.2f\n",t);
				}

				MotionAction *motionAction = static_cast<MotionAction*>(actions[chorI][plans[i]]);
				Trajectory::Ptr motion = motionAction->GetPath();

				if (t >= actions[chorI][plans[i]]->GetEndTime()) {
					if (plans[i] == actions[chorI].size()-1) {

						// Looping code
						// TODO: This needs to effect every drone
						if(looping) {
							this->reset();
							states[i] = StateFlying;
							start = Time::now();
							return;
						}


						states[i] = StateLanding;

						Point lastPoint = motion->evaluate(t).position;
						Point groundPoint(lastPoint.x(), lastPoint.y(), 0);
						double dur = lastPoint.z() / VEHICLE_DESCENT_MS;
						transitions[i] = Trajectory::Ptr( new LinearTrajectory(lastPoint, 0, groundPoint, dur) );
						transitionStarts[i] = Time::now();
						continue;
					}
					plans[i]++;
				} else if (pauseRequested) {
					double nextBreakpoint = getNextBreakpointTime(t);
					if ((int)t + 1 == nextBreakpoint) {
						states[i] = StateHolding;
						pauseIndices[i] = plans[i];
						holdpoints[chorI] = motion->evaluate(t).position;
						continue;
					}
				}

				if(motionAction->GetActionType() == ActionTypes::Hover) {
					hovers[i]->setPoint(motion->evaluate(t).position); // TODO: This line will be a constant for this type of action
					hovers[i]->control(t);
				}
				else {
					posctls[i]->track(motion);
					posctls[i]->control(t);
				}
				for(int j = 0; j < LightController::NUM_LIGHTS; j++){
					//Very important. Must handle the case where you don't specify anything for some of the lights
					//Otherwise it WILL CRASH with an index out of bounds here.
					if(j >= lightActions[chorI].size())
						break;
					int counter = lightCounters[i][j];
					const std::vector<LightAction*> local_action_array = lightActions[chorI][j];
					LightAction* local_action = local_action_array[counter];
					if (counter < local_action_array.size()) {
						auto traj = local_action->GetPath();
						if (t >= local_action->GetStartTime()) {
							lightctls[i]->track(traj, (LightController::LightIndices)j);
							lightctls[i]->control(t);
							if (t >= local_action->GetEndTime()) {
								lightCounters[i][j]++;
							}
						}
					}
				}
			} else if (s == StateLanding) {
				std::vector<int> light_states;
				light_states.resize(LightController::MAX_LIGHTS, 0);
				v.set_lighting(light_states);
				double t = Time::now().since(transitionStarts[i]).seconds();

				// Special behavior near the ground. Stop trying to descend if we are hitting ground effects
				bool groundEffect = (v.state.position.z() < 0.20 && (v.state.velocity.norm() < 0.08)) || v.state.position.z() < 0.15;

				if(t < transitions[i]->endTime() && !groundEffect) {
					posctls[i]->track(transitions[i]);
					posctls[i]->control(t);
				}
				// Disarm
				else if(v.armed){
					v.setpoint_zero();
					v.arm(false);
				}
				// Reset state machine
				else {
					landed = true;
					s = StateInit;
				}

			}
			else if(s == StateFailsafe) {

				// TODO: If still tracking, try a nice controlled landing
				// TODO: Geofence based on covariance to auto-hardkill
				while(v.mode == "offboard") {
					v.set_mode("landing");
				}
			}
		}

		stepTick++;
	}

	void JocsPlayer::prepare() {

		for(const auto& v : vehicles){
			if(!v->tracking) {
				printf("Don't have mocap tracking for at least one drone\n");
				return;
			}
		}

		for(const auto &v : vehicles) {

			// TODO: Also check standard deviation of the position and orientation via mocap data

			Vector3d vec = v->state.orientation.toRotationMatrix() * Vector3d(0, 0, 1);
			if(vec.z() < 0) {
				printf("At least one drone upside down\n");
				return;
			}
			else if(vec.z() < 0.5) {
				printf("At least one drone tilted >45 degrees\n");
				return;
			}

			Time now = Time::now();

			if(now.since(v->onboardState.time).seconds() > 4) {
				printf("Stale onboard orientation feedback\n");
				return;
			}

			if(now.since(v->onboardPositionTime).seconds() > 4) {
				printf("Stale onboard position data\n");
				return;
			}

			// Simulation doesn't have RC input usally
			if(inRealLife && now.since(v->lastRCTime).seconds() > 4) {
				printf("Some drones don't have RC\n");
				return;
			}

			// Compare our state estimate with the onboard state estimate
			// If these deviate by too much, then we probably have a
			Quaterniond d = v->onboardState.orientation * v->state.orientation.inverse();

			if(1.0 - fabs(d.w()) > 0.05) {
				printf("Onboard Pose Not Synced: Too large of a angular deviation\n");
				return;
			}

		}

		for(auto s : states) {
			if(s != StateInit) {
				printf("Cannot prepare: Some drones not in initial state\n");
				return;
			}
		}

		bool inPlace = true;
		int n = jocsActiveIds.size();
		for(int i = 0; i < n; i++) {
			Vector3d start = vehicles[i]->state.position,
					 end = homes[jocsActiveIds[i]];
			end.z() = 0;

			double e = (start - end).norm();
			if(e > 0.5) {
				printf("Role %d not in place. Off by %.2fm\n", i, e);
				inPlace = false;
			}
		}

		if(!inPlace) {
			return;
		}


		for(auto &s : states) {
			s = StateArming;
		}
	}


void JocsPlayer::failsafe() {
	for(auto &s : states) {
		s = StateFailsafe;
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
			timeOffset += Time::now().since(pauseOffset).seconds();
			int n = jocsActiveIds.size();
			for (int i = 0; i < n; i++) {
				plans[i] = pauseIndices[i];
			}
		} else {
			start = Time::now();
			reset();
		}

		for(auto &s : states) {
			s = StateFlying;
		}
	}

	/**
	 * Pause the choreography
	 */
	void JocsPlayer::pause() {
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

			Point lastPoint = holdpoints[jocsActiveIds[i]];
			Point groundPoint = lastPoint; groundPoint.z() = 0;
			double dur = lastPoint.z() / VEHICLE_DESCENT_MS;
			transitions[i] = Trajectory::Ptr( new LinearTrajectory(lastPoint, 0, groundPoint, dur) );
			transitionStarts[i] = Time::now();
		}
	}

	double JocsPlayer::currentTime() {
		if(!this->isPlaying()) {
			return -1;
		}

		return Time::now().since(start).seconds() + startOffset;
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
		for (unsigned i = 0; i < breakpointsLength; i++) {
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
		for (unsigned i = 0; i < breakpointsLength; i++) {
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

	/**
	 * Compute the indices to start and end at given breakpoints
	 * @param startPoint
	 * @param endPoint
	 */
	void JocsPlayer::createBreakpointSection(int startPoint, int endPoint) {
		cout << "createBreakpointSection(" << startPoint << ", " << endPoint << ")" << endl;
		double startTime = -1.0, endTime = -1.0;
		int bpStartIndex = -1 , bpEndIndex = -1;

		vector<int> bpStartIndices(actions.size(), -1);
		vector<int> bpEndIndices(actions.size(), -1);

		if(startPoint >= 0) {
			startTime = this->getBreakpointTime((unsigned)startPoint);
		}

		if(endPoint >= 0) {
			endTime = getBreakpointTime((unsigned)endPoint);
		}

		if (endPoint != -1 && endPoint <= startPoint) {
			printf("Start point must be before endpoint...\n");
			return;
		}

		for (int i = 0; i < actions.size(); i++) {
			for (int j = 0; j < actions[i].size(); j++) {
				if (actions[i][j]->GetStartTime() == startTime) {
					bpStartIndices[i] = j;
				}

				if (actions[i][j]->GetEndTime() == endTime) {
					bpEndIndices[i] = j;
				}

				if (bpStartIndex != -1 && bpEndIndex != -1) {
					break;
				}
			}
		}

		for (int i = 0; i < actions.size(); i++) {
			if (bpStartIndices[i] == -1) {
				bpStartIndices[i] = 0;
			}

			if (bpEndIndices[i] == -1) {
				bpEndIndices[i] = (int)actions[i].size() - 1;
			}
		}

		if (startTime == -1) {
			startTime = 0.0;
		}
		startIndices = bpStartIndices;
		endIndices = bpEndIndices;
		startOffset = startTime;
		cout << "createdBreakpoint section: " << startOffset << endl;
	}

void JocsPlayer::log() {

	double t = Time::now().since(start).seconds();

	logfile << t << ",";
	for(int i = 0; i < vehicles.size(); i++) {
		Vehicle *v = vehicles[i];

		Vector3d target = posctls[i]->getTargetState(t).position;
		logfile << target.x() << "," << target.y() << "," << target.z() << ",";


		Vector3d pos = v->state.position;
		logfile << pos.x() << "," << pos.y() << "," << pos.z() << ",";
	}

	logfile << endl;
	logfile.flush();

}

}
