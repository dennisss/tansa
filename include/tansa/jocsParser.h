//
// Created by kyle on 10/12/16.
//

#ifndef TANSA_JOCSPARSER_H
#define TANSA_JOCSPARSER_H
#include <string>
#include <vector>
#include "json.hpp"
#include "tansa/vehicle.h"
#include "tansa/core.h"
#include "tansa/trajectory.h"

//PLACEHOLDER ACTION CLASS
class Choreography{Choreography(){}};
struct Drone{ Point startingPoint; unsigned id;};
//Static Jocs parsing class for reading actions and drones out of a jocs file
class Jocs{
public:
    static Choreography Parse(const std::string& jocsPath);
private:
    static std::vector<Vehicle> parseVehicles(const nlohmann::json& data);
    static std::vector<int> parseActions(const nlohmann::json& data);
    static Drone parseDrone(nlohmann::json::reference data);
};


#endif //TANSA_JOCSPARSER_H
