//
// Created by kyle on 10/12/16.
//

#include "tansa/jocsParser.h"
#include <fstream>
#include <iostream>
#include <vector>
std::vector<Vehicle> Jocs::parseVehicles(const nlohmann::json &data) {
    //Get array of drones and its size
    auto drones = data["drones"];
    auto length = drones.size();
    for(int i = 0; i < length; i++){
        auto drone = parseDrone(drones[i]);
    }
    //TODO: Fill in this vector
    return std::vector<Vehicle>();
}

std::vector<int> Jocs::parseActions(const nlohmann::json &data) {




}
//Parses a Jocs file
Choreography Jocs::Parse(const std::string &jocsPath) {
    ifstream jocsStream(jocsPath);
    std::string jocsData((std::istreambuf_iterator<char>(jocsStream)), std::istreambuf_iterator<char>());
    auto rawJson = nlohmann::json::parse(jocsData);
    auto actions = parseActions(rawJson);
    auto vehicles = parseVehicles(rawJson);

}

Drone Jocs::parseDrone(nlohmann::json::reference data) {
    Point start;
    start.x() = data["startPosition"][0];
    start.y() = data["startPosition"][1];
    start.z() = data["startPosition"][2];
    unsigned id = data["id"];
    Drone d;
    d.id = id;
    d.startingPoint = start;
    return d;
}
