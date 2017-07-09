#ifndef TANSA_SIMULATE_H
#define TANSA_SIMULATE_H

/** @file simulate.h
 * Classes for running simulations on models
 */

#include "model.h"

#include "core.h"
#include "trajectory.h"
#include "vehicle.h"

#include <pthread.h>

#include <vector>

using namespace std;

namespace tansa {


struct WorldState : State {
	vector<State::Ptr> models;
};

class Firmware;

/**
 * Set of all physical entities in the simulation environment
 */
class World {
public:
	vector<Model::Ptr> models;

	vector<shared_ptr<Firmware>> firmwares;
};


/**
 * The driver class for tracking an evolving world of models
 */
class Simulation {
public:
	static Simulation *Make(Context *ctx);

	Simulation(const DataObject &desc);

	Simulation(World &world, WorldState &state); //json modelDescription);


	/**
	 * Connect a vehicle instance to one of the drones running in the simulator
	 */
	//void connect(Vehicle *v, unsigned i);


	void step(const Time &t);

	void start();
	void stop();

	void track(Vehicle *v, int id) { this->tracked = v; }


	World world;
	WorldState state;

private:

	bool running;
	pthread_t thread;


	Vehicle *tracked = NULL;

	friend void *simulation_thread(void *arg);
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
	Firmware(Context *ctx, const DataObject &desc, MultirotorModel::Ptr model);
	~Firmware();

	void start();
	void stop();

	vector<float> currentActuatorOutputs;

	void connectClient(Vehicle *v);

	Vehicle *getSimVehicle();

	// Given the state of a multi-rotor, this controls it
	void update(State::Ptr s);

private:
	Context *ctx;

	int id;
	string rcScript;


	MultirotorModel::Ptr model;

	int process;

	Vehicle *sim_vehicle;

	void onImuData(const IMUSensorData *data);
	void onGpsData(const GPSData *data);
	void onMocapData(const MocapSensorData *data);
	void onActuatorOutputs(const ActuatorOutputs *actuators);
};



}



#endif
