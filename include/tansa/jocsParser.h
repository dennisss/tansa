//
// Created by kyle on 10/12/16.
//

#ifndef TANSA_JOCSPARSER_H
#define TANSA_JOCSPARSER_H
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include "json.hpp"
#include "tansa/vehicle.h"
#include "tansa/core.h"
#include "tansa/trajectory.h"
#include "tansa/action.h"
namespace tansa {
enum ActionTypes : unsigned{
	Transition = 0,
	Line = 1,
	Circle = 2,
};
//PLACEHOLDER CHOREOGRAPHY CLASS
class Choreography {
	Choreography() {}
};

//Placeholder
struct Drone {
	Drone(Point p, unsigned droneId) : startingPoint(p), id(droneId) {}

	Point startingPoint;
	unsigned id;
};

//Static Jocs parsing class for reading actions and drones out of a jocs file
class Jocs {
public:
	static const std::string HOME_KEY;
	static const std::string DRONE_KEY;
	static const std::string ID_KEY;
	static const std::string CHOREOGRAPHY_KEY;
	static const std::string STARTPOS_KEY;
	static const std::string ENDPOS_KEY;
	static const std::string DURATION_KEY;
	static const std::string ACTION_ROOT_KEY;
	static const std::string ACTION_TIME_KEY;
	static const std::string ACTION_TYPE_KEY;
	static const std::string DRONE_ARRAY_KEY;
	static const std::string ACTION_DATA_KEY;
	static const std::string CIRCLE_ORIGIN_KEY;
	static const std::string CIRCLE_RADIUS_KEY;
	static const std::string CIRCLE_THETA1_KEY;
	static const std::string CIRCLE_THETA2_KEY;

	static Choreography Parse(const std::string &jocsPath);

private:
	static std::vector<Vehicle> parseVehicles(const nlohmann::json &data);

	//placeholder template. This should return action but dont' have action class yet. Was complaining about
	//empty class as template so just used int.
	static void parseActions(const nlohmann::json &data, std::vector<std::unique_ptr<Action>>& actions);

	static Drone parseDrone(const nlohmann::json::reference data);
	static void parseAction(const nlohmann::json::reference data, std::vector<std::unique_ptr<Action>>& actions);
	static ActionTypes convertToActionType(const std::string& data);
};
}
#endif //TANSA_JOCSPARSER_H
