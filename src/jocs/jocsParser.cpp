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

std::vector<std::unique_ptr<Action>> Jocs::Parse(const std::string &jocsPath) {
	ifstream jocsStream(jocsPath);
	std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
	auto rawJson = nlohmann::json::parse(jocsData);
	std::vector<std::unique_ptr<Action>> actions;
	parseActions(rawJson, actions);
	return std::move(actions);
}

void Jocs::parseActions(const nlohmann::json &data, std::vector<std::unique_ptr<Action>>& actions) {
	auto actionsJson = data[CHOREOGRAPHY_KEY];
	//This must be an array
	assert(actionsJson.is_array());

	// This length is temporary but really this is the smallest it would ever be, if only one action is sent at
	// any given time
	auto length = actionsJson.size();
	actions.reserve(length);
	for(int i = 0; i < length; i++){
		parseAction(actionsJson[i], actions);
	}

	// Go back and review actions such that transitions can be created with polynomial trajectories


	//std::cout << actions[3]->GetEndTime() << std::endl; // debug
}

void Jocs::parseAction(const nlohmann::json::reference data, std::vector<std::unique_ptr<Action>>& actions){
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

				actions.push_back(std::move(std::make_unique<MotionAction>(
						drone, std::move(std::make_unique<LinearTrajectory>(start, startTime, end, startTime + duration)))));
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
				actions.push_back(std::move(std::make_unique<MotionAction>(
						drone, std::move(std::make_unique<CircleTrajectory>(origin, radius, theta1, startTime, theta2, startTime + duration)))));
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
