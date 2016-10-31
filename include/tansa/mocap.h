#ifndef TANSA_MOCAP_H_
#define TANSA_MOCAP_H_

#include "vehicle.h"

#include <map>
#include <vector>

class NatNetClient;
struct sFrameOfMocapData;

namespace tansa {

void mocap_callback(struct sFrameOfMocapData* pFrameOfData, void* pUserData);

/**
 * Used to interface pose feedback from motion capture systems with the vehicles
 */
class Mocap {
public:
	Mocap();

	/**
	 * Connect to the motion capture software
	 *
	 * @param iface_addr local ip address of the network interface through which the data is being streamed
	 */
	int connect(string iface_addr, string server_addr);


	int disconnect();

	/**
	 * Explictly specifies that a Vehicle is being tracked by a specific rigid body in the mocap system
	 *
	 * @param v the vehicle being tracked
	 * @param id the id number of the body in Motive
	 */
	void track(Vehicle *v, int id);

private:

	friend void mocap_callback(struct sFrameOfMocapData * pFrameOfData, void* pUserData);

	map<int, Vehicle *> tracked;
	NatNetClient* client;

};


struct RigidBody {
	int id;
	Vector3d position;
	Quaterniond orientation;
	vector<Vector3d> markers;
};

/*
	States:

	int activeVehicle; // Which vehicle is currently being registered


	Mask - get list of pre-existing markers in the scene (0.5 seconds)
	-> Turn led on
	Record - look for new markers (0.5 seconds)
	-> Turn led off
	Scan - look for which of the new markers





*/

/**
 *
 */
struct MocapRegistration {

	Vehicle *vehicle;
	int id;

	bool fixed; // Whether or not the id should be allowed to change

};

struct MocapTrackedMarker {
	Vector3d position;
	Time firstSeen; // TODO: Require multiple frames of visibility to officially consider it tracked
	Time lastSeen; // TODO:

	int count; // Number of times seen/lost
};

/**
 * Higher level
 */
class MocapTracker {
public:

	//void step(const vector<Rigidbody> &bodies, const vector<Vector3d> &markers);


private:

	// List of all pre-existing background markers in the scene which should be ignored
	vector<Vector3d> mask;

	// All new markers
	vector<MocapTrackedMarker> novel;

	//vector<int>

	int selectedVehicle;
	int phase;
	Time start; // When this phase started

};



void correspondence_solve_ideal(const vector<Vector3d> &as, const vector<Vector3d> &bs, vector<unsigned> &c);
void correspondence_arrange(const vector<Vector3d> &as, vector<Vector3d> &out, vector<unsigned> &c);
void rigid_transform_solve(const vector<Vector3d> &as, const vector<Vector3d> &bs, Matrix3d &R, Vector3d &t);

}

#endif
