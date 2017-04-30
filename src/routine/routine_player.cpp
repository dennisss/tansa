#include <tansa/routine.h>

namespace tansa {

// TODO: Instead move logic to manager: Block changes of the program while one is running
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



bool RoutinePlayer::loadChoreography(string jocsPath, float scale, const std::vector<unsigned> &jocsActiveIds, int start){
	cout << "loadJocs(" << jocsPath << ", " << scale << ", " << jocsActiveIds.size() << ", " << start << ")" << endl;
	Routine *choreography = Routine::Load(jocsPath, scale);

	if(!choreography) {
		cout << "Invalid file!" << endl;
		return false;
	}

	return this->loadChoreography(choreography, jocsActiveIds, start);
}

bool RoutinePlayer::loadChoreography(Routine *chor, const std::vector<unsigned> &jocsActiveIds, int start) {
	if (!this->canLoad()) {
		return false;
	}

	delete currentRoutine;
	this->jocsActiveIds = jocsActiveIds;
	landed = false;

	currentRoutine = chor;
	homes = currentRoutine->homes;
	actions = currentRoutine->actions;
	lightActions = currentRoutine->lightActions;
	breakpoints = currentRoutine->breakpoints;

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


std::vector<Breakpoint> RoutinePlayer::getBreakpoints() {
	return breakpoints;
}

std::vector<Point> RoutinePlayer::getHomes() {
	return homes;
}

std::vector<std::vector<Action*>> RoutinePlayer::getActions() {
	return actions;
}


/**
 * Pause the choreography
 */
void RoutinePlayer::pause() {
	pauseRequested = true;
	// TODO: Determine a pause-at index (and maybe also a stop-at index)
}



void RoutinePlayer::reset() {
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


void RoutinePlayer::step(double t) {

	// TODO: This should be a loop over all vehicles with index i

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
			/*
			if(looping) {
				this->reset();
				states[i] = StateFlying;
				start = Time::now();
				return;
			}
			*/


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
}




/**
 * Helper methods
 * TODO: write test cases (especially since the code doesn't use them yet)
 */
double RoutinePlayer::getNextBreakpointTime(double lastTime) {
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
double RoutinePlayer::getBreakpointTime(unsigned breakpointNumber) {
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
double RoutinePlayer::getBreakpointTime(std::string breakpointName) {
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
unsigned RoutinePlayer::getBreakpointNumber(double startTime) {
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
Point RoutinePlayer::getDroneLocationAtTime(double startTime, unsigned droneId) {
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
bool RoutinePlayer::isMotionAction(Action* a) {
	MotionAction* ma = dynamic_cast<MotionAction*>(a);
	return ma;
}

/**
* Compute the indices to start and end at given breakpoints
* @param startPoint
* @param endPoint
*/
void RoutinePlayer::createBreakpointSection(int startPoint, int endPoint) {
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
			bpEndIndices[i] = (int) actions[i].size() - 1;
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

}
