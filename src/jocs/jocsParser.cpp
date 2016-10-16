//
// Created by kyle on 10/12/16.
//
#include "tansa/jocsParser.h"
namespace tansa {
const std::string Jocs::HOME_KEY = "startPosition";
const std::string Jocs::DRONE_KEY = "drones";
const std::string Jocs::ID_KEY = "id";
const std::string Jocs::CHOREOGRAPHY_KEY = "chor";
const std::string Jocs::STARTPOS_KEY = "startPointCenter";
const std::string Jocs::ENDPOS_KEY = "endPointCenter";
const std::string Jocs::DURATION_KEY = "duration";
const std::string Jocs::ACTION_ROOT_KEY = "action";
const std::string Jocs::ACTION_TIME_KEY = "time";
const std::string Jocs::ACTION_TYPE_KEY = "type";
const std::string Jocs::DRONE_ARRAY_KEY = "drones";
const std::string Jocs::ACTION_DATA_KEY = "data";
const std::string Jocs::CIRCLE_ORIGIN_KEY = "originPointCenter";
const std::string Jocs::CIRCLE_RADIUS_KEY = "radius";
const std::string Jocs::CIRCLE_THETA1_KEY = "theta1";
const std::string Jocs::CIRCLE_THETA2_KEY = "theta2";

std::vector< vector<Action*> > Jocs::Parse(const std::string &jocsPath) {
	ifstream jocsStream(jocsPath);
	std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
	auto rawJson = nlohmann::json::parse(jocsData);
	std::vector< vector<Action*> > actions;
	parseActions(rawJson, actions);
	return actions;
}

void Jocs::parseActions(const nlohmann::json &data, std::vector< vector<Action*> >& actions) {
	auto actionsJson = data[CHOREOGRAPHY_KEY];
	//This must be an array
	assert(actionsJson.is_array());

	auto length = actionsJson.size();
	actions.reserve(length);

	// For each timeslot, gather actions in a vector and add as entry in "actions" 2d vector
	for(int i = 0; i < length; i++){
		parseAction(actionsJson[i], actions);
	}

	// Go back and review actions such that transitions can be created with polynomial trajectories
	for(int i = 0; i < actions.size(); i++){

		// Each entry in "actions" has a vector full of the actions that occur at that time
		for (int j=0; j < actions[i].size(); j++){
			if(!actions[i][j]->IsCalculated()){
				if(i == 0){
					//TODO: Handle the case of the first action being a transition where our initial velocity and acceleration are 0 and position is equal to home.
				} else if (i == (actions[i].size() - 1)){
					//TODO: Handle the case of the last action being a transition, where our ending velocity and acceleration are 0 and destination is home.
				} else{
					// Calculate previous and next motion to generate what's needed for the transition
					MotionAction *ref = FindPreviousAction(actions[i][j].GetDrone(), actions, i);
					MotionAction *next = FindNextAction(actions[i][j].GetDrone(), actions, i);
					// TODO: it should be possible that the previous time doesn't have the drone in it,
					// since different timeslots can command different drones. Must continue search to find drone.

					// Get states from the previous and next state that were found
					auto startState = ref->GetPathState(ref->GetEndTime());
					auto endState = next->GetPathState(next->GetStartTime());

					// Store start and end time from EmptyAction to carry over to MotionAction's trajectory
					double thisStart = actions[i][j]->GetStartTime();
					double thisEnd = actions[i][j]->GetEndTime();

					// Cleanup object and replace with a new MotionAction
					delete actions[i][j];
					actions[i][j] = new MotionAction(actions[i][j]->GetDrone(),
													 std::move(std::unique_ptr<PolynomialTrajectory>(
															 PolynomialTrajectory::compute(
																	 {startState.position, startState.velocity,
																	  startState.acceleration},
																	 thisStart,
																	 {endState.position, endState.velocity,
																	  endState.acceleration},
																	 thisEnd))));
				}
			}
		}
	}
}

void Jocs::parseAction(const nlohmann::json::reference data, std::vector<Action*>& actions){
	auto actionsArray = data[ACTION_ROOT_KEY];
	assert(actionsArray.is_array());
	double startTime = data[ACTION_TIME_KEY];
	for(int i = 0; i < actionsArray.size(); i++) {
		auto actionsArrayElement = actionsArray[i];
		double duration = actionsArrayElement[DURATION_KEY];
		unsigned type = convertToActionType(actionsArrayElement[ACTION_TYPE_KEY]);
		//TODO: Assuming no grouping right now. Will have to make this a loop to create actions for each drone and calc offset
		assert(actionsArrayElement[DRONE_ARRAY_KEY].is_array());
		unsigned drone = actionsArrayElement[DRONE_ARRAY_KEY][0];
		//Switch on type of Action
		switch (type) {
			//Transitional case: Put a placeholder action to be post-processed away
			case ActionTypes::Transition: {
				// Actual calculation will be processed after this loop
				// For now, there is no trajectory !
				actions.push_back(new EmptyAction(drone, startTime, startTime + duration));
				break;
			}
			//Simple line action
			case ActionTypes::Line: {
				Point start(
						actionsArrayElement[ACTION_DATA_KEY][STARTPOS_KEY][0],
						actionsArrayElement[ACTION_DATA_KEY][STARTPOS_KEY][1],
						actionsArrayElement[ACTION_DATA_KEY][STARTPOS_KEY][2]);

				Point end(
						actionsArrayElement[ACTION_DATA_KEY][ENDPOS_KEY][0],
						actionsArrayElement[ACTION_DATA_KEY][ENDPOS_KEY][1],
						actionsArrayElement[ACTION_DATA_KEY][ENDPOS_KEY][2]);

				actions.push_back(new MotionAction(
						drone, std::move(std::make_unique<LinearTrajectory>(start, startTime, end, startTime + duration))));
				break;
			}
			//Circle Action
			case ActionTypes::Circle: {
				Point origin(
						actionsArrayElement[ACTION_DATA_KEY][CIRCLE_ORIGIN_KEY][0],
						actionsArrayElement[ACTION_DATA_KEY][CIRCLE_ORIGIN_KEY][1],
						actionsArrayElement[ACTION_DATA_KEY][CIRCLE_ORIGIN_KEY][2]);
				double radius = actionsArrayElement[ACTION_DATA_KEY][CIRCLE_RADIUS_KEY];
				double theta1 = actionsArrayElement[ACTION_DATA_KEY][CIRCLE_THETA1_KEY];
				double theta2 = actionsArrayElement[ACTION_DATA_KEY][CIRCLE_THETA2_KEY];
				actions.push_back(new MotionAction(
						drone, std::move(std::make_unique<CircleTrajectory>(origin, radius, theta1, startTime, theta2, startTime + duration))));
				break;
			}
			default: {
				break;
			}
		}
	}
}

ActionTypes Jocs::convertToActionType(const std::string& data){
	if(data == "transition")
		return ActionTypes::Transition;
	else if (data == "line")
		return ActionTypes::Line;
	else if (data == "circle")
		return ActionTypes::Circle;
}

//TODO: maybe instead of returning a pointer we take it as a parameter which we modify?
MotionAction* Jocs::FindPreviousAction(DroneId id, std::vector< vector<Action*> >& actions, int currentLocation) {
	for(int i = currentLocation-1; i >= 0; i--){
		MotionAction* actionFound = nullptr;
		bool success = FindMotionForDrone(id, actions[i], actionFound);
		if (success) {
			return actionFound;
		}
	}
	// This means there never *was* a previous motion for this drone
	//TODO: maybe should throw exception here?
	return nullptr;
}

//TODO: maybe instead of returning a pointer we take it as a parameter which we modify?
MotionAction* Jocs::FindNextAction(DroneId id, std::vector< vector<Action*> >& actions, int currentLocation) {
	for(int i = currentLocation+1; i < actions.size(); i++){
		MotionAction* actionFound = nullptr;
		bool success = FindMotionForDrone(id, actions[i], actionFound);
		if (success) {
			return actionFound;
		}
	}
	// This means there never *will be* a next motion for this drone
	//TODO: maybe should throw exception here?
	return nullptr;
}

bool Jocs::FindMotionForDrone(DroneId id, vector<Action*> curActions, MotionAction& actionFound) {
	for(int i = 0; i < curActions.size(); i++){
		if (curActions[i].GetDrone() == id) {
			// We found the drone we were looking for *wink wink*
			actionFound = static_cast<MotionAction*>(curActions[i]);
			return true;
		}
	}
	return false; // This means that the previous time didn't have the drone in it.
}
}
