#ifndef TANSA_SIMULATE_H
#define TANSA_SIMULATE_H

/** @file simulate.h
 * Classes for running simulations on models
 */

#include "model.h"

#include "core.h"
#include "trajectory.h"
#include "vehicle.h"

#include <vector>

using namespace std;

namespace tansa {


struct WorldState : State {
	vector<State::Ptr> models;
};

/**
 * Set of all physical entities in the simulation environment
 */
class World {
public:
	vector<Model::Ptr> models;
};


/**
 * The driver class for tracking an evolving world of models
 */
class Simulation {
public:
	Simulation(const DataObject &desc);

	Simulation(World &world, WorldState &state); //json modelDescription);


	/**
	 * Connect a vehicle instance to one of the drones running in the simulator
	 */
	//void connect(Vehicle *v, unsigned i);


	void step();

	void start();


	World world;
	WorldState state;

};




/**
 * Manages an SITL instance alongside simulations and communication to it
 */
class Firmware {
public:

	/*
		Defined in terms of:
		- rcS base script
		- id/number -> this defines the ports
			- pair of mavlink ports with simulator interface
			- pair of mavlink ports for regular interface
	*/
	Firmware(MultirotorModel::Ptr model);


	void start();
	void stop();

	vector<float> currentActuatorOutputs;

	void connectClient(Vehicle *v);

	Vehicle *getSimVehicle();

private:

	int id;
	string rcScript;


	int process;

	Vehicle *sim_vehicle;

	void onImuData(const IMUSensorData *data);

	void onActuatorOutputs(const ActuatorOutputs *actuators);
};



}



#endif
