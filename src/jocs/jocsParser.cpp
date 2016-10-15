//
// Created by kyle on 10/12/16.
//
#include "tansa/jocsParser.h"
namespace tansa {
const std::string Jocs::HOME_KEY = "startPosition";
const std::string Jocs::DRONE_KEY = "drones";
const std::string Jocs::ID_KEY = "id";
const std::string Jocs::CHOREOGRAPHY_KEY = "chor";
const std::string Jocs::STARTPOS_KEY = "start";
const std::string Jocs::ENDPOS_KEY = "end";
const std::string Jocs::DURATION_KEY = "duration";
const std::string Jocs::ACTION_ROOT_KEY = "action";
const std::string Jocs::ACTION_TIME_KEY = "time";
const std::string Jocs::ACTION_TYPE_KEY = "type";
const std::string Jocs::DRONE_ARRAY_KEY = "drones";
const std::string Jocs::ACTION_DATA_KEY = "data";
const std::string Jocs::CIRCLE_ORIGIN_KEY = "origin";
const std::string Jocs::CIRCLE_RADIUS_KEY = "radius";
const std::string Jocs::CIRCLE_THETA1_KEY = "theta1";
const std::string Jocs::CIRCLE_THETA2_KEY = "theta2";

//Parses a Jocs file
Choreography Jocs::Parse(const std::string &jocsPath) {
	ifstream jocsStream(jocsPath);
	std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
	auto rawJson = nlohmann::json::parse(jocsData);
	std::vector<std::unique_ptr<Action>> actions;
	parseActions(rawJson, actions);
}
//On hold for now. Still working on how to represent vehicles and grouping
std::vector<Vehicle> Jocs::parseVehicles(const nlohmann::json &data) {
	//Get array of drones and its size
	auto drones = data[DRONE_KEY];
	auto length = drones.size();
	for (int i = 0; i < length; i++) {
		//TODO: Actually create vehicle here instead of this placeholder drone object
		auto drone = parseDrone(drones[i]);
	}
	//TODO: Fill in this vector
	return std::vector<Vehicle>();
}

// Input will be the whole jocs file body under "chor" section of json
// Output will be vector of actions
void Jocs::parseActions(const nlohmann::json &data, std::vector<std::unique_ptr<Action>>& actions) {
	auto actionsJson = data[CHOREOGRAPHY_KEY];
	//This must be an array
	assert(actionsJson.is_array());
	auto length = actionsJson.size();
	actions.reserve(length);
	for(int i = 0; i < length; i++){
		parseAction(actionsJson[i], actions);
	}
	std::cout << actions[3]->GetEndTime() << std::endl;
}

Drone Jocs::parseDrone(const nlohmann::json::reference data) {
	Point start(data[HOME_KEY][0], data[HOME_KEY][1], data[HOME_KEY][2]);
	Drone d(start, data[ID_KEY]);
	return d;
}
//Untested, but it compiles. Example way of parsing jocs actions.
void Jocs::parseAction(const nlohmann::json::reference data, std::vector<std::unique_ptr<Action>>& actions){

	auto actionsArray = data[ACTION_ROOT_KEY];
	assert(actionsArray.is_array());
	double startTime = data[ACTION_TIME_KEY];
	for(int i = 0; i < actionsArray.size(); i++) {
		auto actionsArrayElement = actionsArray[i];
		unsigned type = convertToActionType(actionsArrayElement[ACTION_TYPE_KEY]);
		//TODO: Assuming no grouping right now. Will have to adjust this with groups
		unsigned drone = actionsArrayElement[DRONE_ARRAY_KEY][0];
		//Switch on type of Action
		switch (type) {
			//Transitional case: Put a placeholder action to be post-processed away
			case ActionTypes::Transition: {
				// Actual calculation will be processed after this loop
				// For now, there is no trajectory !
				double duration = actionsArrayElement[DURATION_KEY]; // This will be taken out of the switch after merge

				actions.push_back(std::move(std::make_unique<EmptyAction>(drone, startTime, startTime + duration)));

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

				double duration = actionsArrayElement[DURATION_KEY];

				actions.push_back(std::move(std::make_unique<MotionAction>(
						drone, std::make_unique<LinearTrajectory>(start, startTime, end, startTime + duration))));
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
				double duration = actionsArrayElement[DURATION_KEY];
				actions.push_back(std::move(std::make_unique<MotionAction>(
						drone, std::make_unique<CircleTrajectory>(origin, radius, theta1, startTime, theta2, startTime + duration))));
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
}
