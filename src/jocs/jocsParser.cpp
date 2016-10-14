//
// Created by kyle on 10/12/16.
//
#include "tansa/jocsParser.h"

const std::string Jocs::HOME_KEY = "startPosition";
const std::string Jocs::DRONE_KEY = "drones";
//Parses a Jocs file
Choreography Jocs::Parse(const std::string &jocsPath) {
	ifstream jocsStream(jocsPath);
	std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
	auto rawJson = nlohmann::json::parse(jocsData);
	auto actions = parseActions(rawJson);
	auto vehicles = parseVehicles(rawJson);
}

std::vector<Vehicle> Jocs::parseVehicles(const nlohmann::json &data) {
	//Get array of drones and its size
	auto drones = data[DRONE_KEY];
	auto length = drones.size();
	for(int i = 0; i < length; i++){
		//TODO: Actually create vehicle here instead of this placeholder drone object
		auto drone = parseDrone(drones[i]);
	}
	//TODO: Fill in this vector
	return std::vector<Vehicle>();
}
//TODO: implement this. will be similar to parse vehicle
// Input will be the whole jocs file body under "chor" section of json
// Output will be vector of actions
std::vector<int> Jocs::parseActions(const nlohmann::json &data) {

}

Drone Jocs::parseDrone(nlohmann::json::reference data) {
	Point start(data[HOME_KEY][0],data[HOME_KEY][1],data[HOME_KEY][2]);
	Drone d(start, data["id"]);
	return d;
}
