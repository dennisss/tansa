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
const std::string Jocs::UNITS_KEY = "units";

const double Jocs::FEET_TO_METERS = 0.3048;
const double Jocs::DEGREES_TO_RADIANS = M_PI/180.0;

std::vector< vector<Action*> > Jocs::Parse() {
	ifstream jocsStream(jocsPath);
	std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
	auto rawJson = nlohmann::json::parse(jocsData);
	auto units = rawJson[UNITS_KEY];
	needConvertToMeters = (units["length"] == "feet");
	needConvertToRadians = (units["angle"] == "degrees");
	std::vector< vector<Action*> > actions;
	parseActions(rawJson, actions);
	return actions;
}

void Jocs::parseActions(const nlohmann::json &data, std::vector< vector<Action*> >& actions) {
	auto actionsJson = data[CHOREOGRAPHY_KEY];
	auto drones = data[DRONE_ARRAY_KEY];

	assert(drones.is_array());
	//This must be an array
	assert(actionsJson.is_array());
	auto droneLength = drones.size();
	homes.resize(droneLength);
	for(int i = 0; i < droneLength; i++){
		homes[i] = Point(drones[i][HOME_KEY][0], drones[i][HOME_KEY][1], drones[i][HOME_KEY][2]);
	}
	auto length = actionsJson.size();
	//allocate space for each subarray for each drone.
	actions.resize(droneLength);
	// For each drone, gather actions in a vector and add as entry in "actions" 2d vector
	for(int i = 0; i < length; i++){
		parseAction(actionsJson[i], actions);
	}
	// Go back and review actions such that transitions can be created with polynomial trajectories
	for(int i = 0; i < actions.size(); i++){
		// Each entry in "actions" has a vector full of actions for that drone
		for (int j=0; j < actions[i].size(); j++){
			if(!actions[i][j]->IsCalculated()){
				double thisStart = actions[i][j]->GetStartTime();
				double thisEnd = actions[i][j]->GetEndTime();
				if(j == 0){
						MotionAction *next = static_cast<MotionAction *>(actions[i][j + 1]);
						auto endState = next->GetPathState(next->GetStartTime());
						std::cout << endState.velocity << std::endl;
						delete actions[i][j];
						actions[i][j] = new MotionAction(i,
							 PolynomialTrajectory::compute(
									 {homes[i]},
									 thisStart,
									 {endState.position, endState.velocity,
									  endState.acceleration},
									 thisEnd
							 ),
							 ActionTypes::Transition
						);
				} else if (j == (actions[i].size() - 1)){
					//TODO: Handle the case of the last action being a transition, where our ending velocity and acceleration are 0 and destination is home.
				} else{

						// Calculate previous and next motion to generate what's needed for the transition
						MotionAction *prev = static_cast<MotionAction *>(actions[i][j - 1]);
						MotionAction *next = static_cast<MotionAction *>(actions[i][j + 1]);
						// Get states from the previous and next state that were found
						auto startState = prev->GetPathState(prev->GetEndTime());
						auto endState = next->GetPathState(next->GetStartTime());
						// Cleanup object and replace with a new MotionAction
						delete actions[i][j];
						actions[i][j] = new MotionAction(i, PolynomialTrajectory::compute(
								{startState.position, startState.velocity, startState.acceleration},
								thisStart,
								{endState.position, endState.velocity, endState.acceleration},
								thisEnd
														 ),
								ActionTypes::Transition
						);
				}
			}
		}
	}
}

void Jocs::parseAction(nlohmann::json::reference data, std::vector<std::vector<Action*>>& actions){
	auto actionsArray = data[ACTION_ROOT_KEY];
	assert(actionsArray.is_array());
	double startTime = data[ACTION_TIME_KEY];
	for(int i = 0; i < actionsArray.size(); i++) {
		auto actionsArrayElement = actionsArray[i];
		double duration = actionsArrayElement[DURATION_KEY];
		unsigned type = convertToActionType(actionsArrayElement[ACTION_TYPE_KEY]);
		assert(actionsArrayElement[DRONE_ARRAY_KEY].is_array());
		auto drones = actionsArrayElement[DRONE_ARRAY_KEY];
		double conversionFactor = needConvertToMeters ? FEET_TO_METERS : 1.0;
		assert(drones.is_array());
		for (int j = 0; j < drones.size(); j++) {
			unsigned drone = drones[j][ID_KEY];
			Point offset(drones[j]["offset"][0],drones[j]["offset"][1],drones[j]["offset"][2]);
			switch (type) {
				case ActionTypes::Transition: {
					// Actual calculation will be processed after this loop
					actions[drone].push_back(new EmptyAction(drone, startTime, startTime + duration));
					break;
				}
				case ActionTypes::Line: {
					Point start(
							actionsArrayElement[ACTION_DATA_KEY][STARTPOS_KEY][0],
							actionsArrayElement[ACTION_DATA_KEY][STARTPOS_KEY][1],
							actionsArrayElement[ACTION_DATA_KEY][STARTPOS_KEY][2]);
					start*=conversionFactor;
					Point end(
							actionsArrayElement[ACTION_DATA_KEY][ENDPOS_KEY][0],
							actionsArrayElement[ACTION_DATA_KEY][ENDPOS_KEY][1],
							actionsArrayElement[ACTION_DATA_KEY][ENDPOS_KEY][2]);
					end*=conversionFactor;
					actions[drone].push_back(new MotionAction(
							drone, new LinearTrajectory(start + offset, startTime, end + offset, startTime + duration), ActionTypes::Line));
					break;
				}
				case ActionTypes::Circle: {
					Point origin(
							actionsArrayElement[ACTION_DATA_KEY][CIRCLE_ORIGIN_KEY][0],
							actionsArrayElement[ACTION_DATA_KEY][CIRCLE_ORIGIN_KEY][1],
							actionsArrayElement[ACTION_DATA_KEY][CIRCLE_ORIGIN_KEY][2]);
					origin*=conversionFactor;
					double radius = actionsArrayElement[ACTION_DATA_KEY][CIRCLE_RADIUS_KEY];
					double theta1 = actionsArrayElement[ACTION_DATA_KEY][CIRCLE_THETA1_KEY];
					radius*=conversionFactor;
					double theta2 = actionsArrayElement[ACTION_DATA_KEY][CIRCLE_THETA2_KEY];
					if(needConvertToRadians) {
						theta1 = theta1 * DEGREES_TO_RADIANS;
						theta2 = theta2 * DEGREES_TO_RADIANS;
					}
					actions[drone].push_back(new MotionAction(
							drone,
							new CircleTrajectory(origin + offset, radius, theta1, startTime, theta2, startTime + duration),
							ActionTypes::Circle
					));
					break;
				}
				case ActionTypes::Hover:{
					Point hoverPoint(
							actionsArrayElement[ACTION_DATA_KEY]["startPointCenter"][0],
							actionsArrayElement[ACTION_DATA_KEY]["startPointCenter"][1],
							actionsArrayElement[ACTION_DATA_KEY]["startPointCenter"][2]
					);
					hoverPoint*=conversionFactor;
					actions[drone].push_back(new MotionAction(
							drone,
							new LinearTrajectory(hoverPoint + offset, startTime, hoverPoint + offset, startTime + duration),
							ActionTypes::Hover
					));
					break;
				}
				default: {
					break;
				}
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
	else if (data == "hover")
		return ActionTypes::Hover;
}
}
