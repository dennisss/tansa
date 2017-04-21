#include "tansa/jocsParser.h"
#include "tansa/breakpoint.h"
#include <regex>
#include <algorithm>
#include <stdexcept>
namespace tansa {
const std::string Jocs::HOME_KEY = "startPosition";
const std::string Jocs::ID_KEY = "id";

const std::string Jocs::UNITS_KEY = "units";
const std::string Jocs::LENGTH_KEY = "length";
const std::string Jocs::TIME_KEY = "time";
const std::string Jocs::ANGLE_KEY = "angle";

const std::string Jocs::THEATER_VOLUME_KEY = "theater_volume";
const std::string Jocs::MIN_KEY = "min";
const std::string Jocs::MAX_KEY = "max";

const std::string Jocs::BREAK_KEY = "breakpoints";
const std::string Jocs::BREAK_NAME = "name";
const std::string Jocs::BREAK_START = "startTime";
const std::string Jocs::BREAK_NUMBER = "number";

const std::string Jocs::REPEAT_KEY = "repeat";

const std::string Jocs::CHOREOGRAPHY_KEY = "chor";
const std::string Jocs::ACTION_TIME_KEY = "time";

const std::string Jocs::ACTION_ROOT_KEY = "action";
const std::string Jocs::ACTION_TYPE_KEY = "type";
const std::string Jocs::DURATION_KEY = "duration";

const std::string Jocs::DRONE_ARRAY_KEY = "drones";
const std::string Jocs::DRONE_START_OFF_KEY = "startOffset";
const std::string Jocs::DRONE_END_OFF_KEY = "endOffset";
const std::string Jocs::DRONE_OFF_KEY = "totalOffset"; // TODO: use this for circles and hovers

const std::string Jocs::ACTION_DATA_KEY = "data";
const std::string Jocs::STARTPOS_KEY = "startPoint";
const std::string Jocs::ENDPOS_KEY = "endPoint";
const std::string Jocs::HOVER_KEY = "hoverPoint";
const std::string Jocs::CIRCLE_ORIGIN_KEY = "originPoint";
const std::string Jocs::CIRCLE_RADIUS_KEY = "radius";
const std::string Jocs::CIRCLE_THETA1_KEY = "theta1";
const std::string Jocs::CIRCLE_THETA2_KEY = "theta2";

const std::string Jocs::START_INTENSITY_KEY = "startIntensity";
const std::string Jocs::END_INTENSITY_KEY = "endIntensity";
const std::string Jocs::BPS_KEY = "bps";

const double Jocs::FEET_TO_METERS = 0.3048;
const double Jocs::DEGREES_TO_RADIANS = M_PI/180.0;

Jocs::~Jocs() {
	for (const auto &vec : actions) {
		for (const auto &a : vec) {
			delete a;
		}
	}
	for (const auto &light : lightActions) {
		for (const auto &drone : light) {
			for(const auto& action: drone){
				delete action;
			}
		}
	}
}

Jocs* Jocs::Parse(std::string jocsPath, double scale) {
	json rawJson = DataObject::LoadFile(jocsPath);
	auto units = rawJson[UNITS_KEY];
	bool needConvertToMeters = (units[LENGTH_KEY] == "feet");
	bool needConvertToRadians = (units[ANGLE_KEY] == "degrees");

	unsigned repeat = rawJson[REPEAT_KEY];
	auto ret = new Jocs(needConvertToMeters, needConvertToRadians, repeat);

	ret->parseActions(rawJson, scale);
	ret->parseBreakpoints(rawJson);

	return ret;
}

void Jocs::parseActions(const nlohmann::json &data, double scale) {
	// TODO: split this into parseActions and parseDrones

	auto actionsJson = data[CHOREOGRAPHY_KEY];
	auto drones = data[DRONE_ARRAY_KEY];
	assert(drones.is_array());
	assert(actionsJson.is_array());
	auto droneLength = drones.size();
	homes.resize(droneLength);
	double conversionFactor = needConvertToMeters ? FEET_TO_METERS : 1.0;
	for(unsigned i = 0; i < droneLength; i++){
		homes[i] = Point(drones[i][HOME_KEY][0], drones[i][HOME_KEY][1], drones[i][HOME_KEY][2])*conversionFactor*(scale);
	}

	auto length = actionsJson.size();
	//allocate space for each subarray for each drone.
	actions.resize(droneLength);
	lightActions.resize(droneLength);
	// For each drone, gather actions in a vector and add as entry in "actions" 2d vector
	double lastTime = 0.0;
	for(unsigned k = 0; k < repeat; k++) {
		for (unsigned i = 0; i < length; i++) {
			if (i == length - 1) {
				lastTime = parseAction(actionsJson[i], lastTime, scale);
			} else {
				parseAction(actionsJson[i], lastTime, scale);
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
					// - can safely assume all are motions since made separate array for lights
					MotionAction* prev = dynamic_cast<MotionAction*>(actions[i][j - 1]);
					MotionAction* next = dynamic_cast<MotionAction*>(actions[i][j + 1]);

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
double Jocs::parseAction(const nlohmann::json::reference data, double lastTime, double scale){
	auto actionsArray = data[ACTION_ROOT_KEY];
	assert(actionsArray.is_array());
	double startTime = data[ACTION_TIME_KEY];
	double endTime = 0.0;

	for(unsigned i = 0; i < actionsArray.size(); i++) {
		auto actionsArrayElement = actionsArray[i];

		double duration = actionsArrayElement[DURATION_KEY];

		ActionTypes type = convertToActionType(actionsArrayElement[ACTION_TYPE_KEY]);
		assert(actionsArrayElement[DRONE_ARRAY_KEY].is_array());

		auto drones = actionsArrayElement[DRONE_ARRAY_KEY];
		double conversionFactor = needConvertToMeters ? FEET_TO_METERS : 1.0;
		conversionFactor *= scale;
		endTime += lastTime + startTime + duration;
		assert(drones.is_array());

		for (unsigned j = 0; j < drones.size(); j++) {
			unsigned drone = drones[j][ID_KEY];

			if (type == ActionTypes::Light) {

				/*// Get the lights that this action applies to
				//auto lights = actionsArrayElement[ACTION_DATA_KEY][LIGHTS_INCL_KEY];
				//assert(lights.is_array());

				// TODO: Create separate actions - one for each light for each drone
				// UNLESS if we decide to still use one set_lighting command for all lights,
				// simply use two "paths" in the "Trajectory" class.

				float si = actionsArrayElement[ACTION_DATA_KEY][START_INTENSITY_KEY];
				float ei = actionsArrayElement[ACTION_DATA_KEY][END_INTENSITY_KEY];
				lightActions[0][drone].push_back(new LightAction(
						drone,
						make_shared<LightTrajectory>(
								si,
								lastTime + startTime,
								ei,
								lastTime + startTime + duration)
				));*/
			} else if (type == ActionTypes::Strobe) {

				/*float si = actionsArrayElement[ACTION_DATA_KEY][START_INTENSITY_KEY];
				float ei = actionsArrayElement[ACTION_DATA_KEY][END_INTENSITY_KEY];
				float bps = actionsArrayElement[ACTION_DATA_KEY][BPS_KEY];
				//NOTE this is hack to get this to compile. JOCS only handled 1 light
				lightActions[0][drone].push_back(new LightAction(
						drone,
						make_shared<StrobeTrajectory>(
								si,
								lastTime + startTime,
								ei,
								lastTime + startTime + duration,
								bps)
				));*/
			} else {
				Point startOffset(drones[j][DRONE_START_OFF_KEY][0], drones[j][DRONE_START_OFF_KEY][1], drones[j][DRONE_START_OFF_KEY][2]);
				Point endOffset(drones[j][DRONE_END_OFF_KEY][0], drones[j][DRONE_END_OFF_KEY][1], drones[j][DRONE_END_OFF_KEY][2]);
				startOffset*=conversionFactor;
				endOffset*=conversionFactor;
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
								drone, Trajectory::Ptr(new LinearTrajectory(start + startOffset, lastTime + startTime, end + endOffset, lastTime + startTime + duration)), ActionTypes::Line));
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
						// TODO: use separate offset key for circle and hover
						actions[drone].push_back(new MotionAction(
								drone,
								Trajectory::Ptr(new CircleTrajectory(origin + startOffset, radius, theta1, lastTime+startTime, theta2, lastTime + startTime + duration)),
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
						// TODO: use separate offset key for circle and hover
						actions[drone].push_back(new MotionAction(
								drone,
								Trajectory::Ptr(new LinearTrajectory(hoverPoint + startOffset, lastTime +  startTime, hoverPoint + endOffset, lastTime + startTime + duration)),
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
	return endTime;
}

void Jocs::parseBreakpoints(const nlohmann::json &data) {

	// Skip parsing if not defined in file
	if(data.count(BREAK_KEY) != 1) {
		return;
	}

	auto breakpointsRaw = data[BREAK_KEY];

	assert(breakpointsRaw.is_array());

	auto breakpointsLength = breakpointsRaw.size();

	for(unsigned i = 0; i < breakpointsLength; i++){
		breakpoints.push_back(Breakpoint(breakpointsRaw[i][BREAK_NAME],
											 (unsigned)breakpointsRaw[i][BREAK_NUMBER],
											 (double)breakpointsRaw[i][BREAK_START]));
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
	else if (data == "light")
		return ActionTypes::Light;
	else if (data == "strobe")
		return ActionTypes::Strobe;
	return ActionTypes::None;
}

}
