#include "tansa/jocsParser.h"
#include <regex>
namespace tansa {
const std::string Jocs::DRONE_KEY = "drones";
const std::string Jocs::HOME_KEY = "startPosition";
const std::string Jocs::ID_KEY = "id";

const std::string Jocs::UNITS_KEY = "units";
const std::string Jocs::LENGTH_KEY = "length";
const std::string Jocs::TIME_KEY = "time";
const std::string Jocs::ANGLE_KEY = "angle";

const std::string Jocs::THEATER_VOLUME_KEY = "theater_volume";
const std::string Jocs::MIN_KEY = "min";
const std::string Jocs::MAX_KEY = "max";

const std::string Jocs::REPEAT_KEY = "repeat";

const std::string Jocs::CHOREOGRAPHY_KEY = "chor";
const std::string Jocs::ACTION_TIME_KEY = "time";

const std::string Jocs::ACTION_ROOT_KEY = "action";
const std::string Jocs::ACTION_TYPE_KEY = "type";
const std::string Jocs::DURATION_KEY = "duration";

const std::string Jocs::DRONE_ARRAY_KEY = "drones";
const std::string Jocs::DRONE_START_OFF_KEY = "startOffset";
const std::string Jocs::DRONE_END_OFF_KEY = "endOffset";

const std::string Jocs::ACTION_DATA_KEY = "data";
const std::string Jocs::STARTPOS_KEY = "startPoint";
const std::string Jocs::ENDPOS_KEY = "endPoint";
const std::string Jocs::HOVER_KEY = "hoverPoint";
const std::string Jocs::CIRCLE_ORIGIN_KEY = "originPoint";
const std::string Jocs::CIRCLE_RADIUS_KEY = "radius";
const std::string Jocs::CIRCLE_THETA1_KEY = "theta1";
const std::string Jocs::CIRCLE_THETA2_KEY = "theta2";

const double Jocs::FEET_TO_METERS = 0.3048;
const double Jocs::DEGREES_TO_RADIANS = M_PI/180.0;

Jocs Jocs::Parse(std::string jocsPath) {
	ifstream jocsStream(jocsPath);
	std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
	//For some reason this regex didn't like the end of line $...but does work without it
	assert(jocsData.find("//") == std::string::npos);
	auto rawJson = nlohmann::json::parse(jocsData);
	auto units = rawJson[UNITS_KEY];
	bool needConvertToMeters = (units[LENGTH_KEY] == "feet");
	bool needConvertToRadians = (units[ANGLE_KEY] == "degrees");
	unsigned repeat = rawJson[REPEAT_KEY];
	auto ret = Jocs(needConvertToMeters, needConvertToRadians, repeat);
	ret.parseActions(rawJson);
	return ret;
}

void Jocs::parseActions(const nlohmann::json &data) {

	auto actionsJson = data[CHOREOGRAPHY_KEY];
	auto drones = data[DRONE_ARRAY_KEY];
	assert(drones.is_array());
	assert(actionsJson.is_array());
	auto droneLength = drones.size();
	homes.resize(droneLength);
	double conversionFactor = needConvertToMeters ? FEET_TO_METERS : 1.0;
	for(unsigned i = 0; i < droneLength; i++){
		homes[i] = Point(drones[i][HOME_KEY][0], drones[i][HOME_KEY][1], drones[i][HOME_KEY][2])*conversionFactor;
	}

	auto length = actionsJson.size();
	//allocate space for each subarray for each drone.
	actions.resize(droneLength);
	// For each drone, gather actions in a vector and add as entry in "actions" 2d vector
	double lastTime = 0.0;
	for(unsigned k = 0; k < repeat; k++) {
		for (unsigned i = 0; i < length; i++) {
			if (i == length - 1) {
				lastTime = parseAction(actionsJson[i], lastTime);
			} else {
				parseAction(actionsJson[i], lastTime);
			}
		}
	}
	// Go back and review actions such that transitions can be created with polynomial trajectories
	for (unsigned i = 0; i < actions.size(); i++) {
		// Each entry in "actions" has a vector full of actions for that drone
		for (unsigned j = 0; j < actions[i].size(); j++) {
			if (!actions[i][j]->IsCalculated()) {
				double thisStart = actions[i][j]->GetStartTime();
				double thisEnd = actions[i][j]->GetEndTime();
				if (j == 0) {
					MotionAction *next = static_cast<MotionAction *>(actions[i][j + 1]);
					auto endState = next->GetPathState(next->GetStartTime());
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
				} else if (j == (actions[i].size() - 1)) {
					//TODO: Handle the case of the last action being a transition, where our ending velocity and acceleration are 0 and destination is home.
				} else {

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

// TODO: rename since it parses all actions at given timeslot
double Jocs::parseAction(const nlohmann::json::reference data, double lastTime){
	auto actionsArray = data[ACTION_ROOT_KEY];
	assert(actionsArray.is_array());
	double startTime = data[ACTION_TIME_KEY];
	double endTime = 0.0;

	for(unsigned i = 0; i < actionsArray.size(); i++) {
		auto actionsArrayElement = actionsArray[i];

		double duration = actionsArrayElement[DURATION_KEY];

		unsigned type = convertToActionType(actionsArrayElement[ACTION_TYPE_KEY]);
		assert(actionsArrayElement[DRONE_ARRAY_KEY].is_array());

		auto drones = actionsArrayElement[DRONE_ARRAY_KEY];
		double conversionFactor = needConvertToMeters ? FEET_TO_METERS : 1.0;
		endTime += lastTime + startTime + duration;
		assert(drones.is_array());

		for (unsigned j = 0; j < drones.size(); j++) {
			unsigned drone = drones[j][ID_KEY];
			Point offset(drones[j][DRONE_START_OFF_KEY][0],drones[j][DRONE_START_OFF_KEY][1],drones[j][DRONE_START_OFF_KEY][2]);
			offset*=conversionFactor;
			switch (type) {
				case ActionTypes::Transition: {
					// Actual calculation will be processed after this loop
					actions[drone].push_back(new EmptyAction(drone, lastTime + startTime, lastTime + startTime + duration));
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
							drone, new LinearTrajectory(start + offset, lastTime + startTime, end + offset, lastTime + startTime + duration), ActionTypes::Line));
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
							new CircleTrajectory(origin + offset, radius, theta1, lastTime+startTime, theta2, lastTime + startTime + duration),
							ActionTypes::Circle
					));
					break;
				}
				case ActionTypes::Hover:{
					Point hoverPoint(
							actionsArrayElement[ACTION_DATA_KEY][HOVER_KEY][0],
							actionsArrayElement[ACTION_DATA_KEY][HOVER_KEY][1],
							actionsArrayElement[ACTION_DATA_KEY][HOVER_KEY][2]
					);
					hoverPoint*=conversionFactor;
					actions[drone].push_back(new MotionAction(
							drone,
							new LinearTrajectory(hoverPoint + offset, lastTime +  startTime, hoverPoint + offset, lastTime + startTime + duration),
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
	return endTime;
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
	return ActionTypes::None;
}
}
