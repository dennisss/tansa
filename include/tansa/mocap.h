#ifndef TANSA_MOCAP_H_
#define TANSA_MOCAP_H_

#include "vehicle.h"

#include <map>
#include <vector>

namespace tansa {

namespace optitrack {
class NatNetClient;
struct NatNetFrame;
}

class RigidBodyTracker;

enum MocapMode {
	MocapPassthrough,
	MocapRigidBodyFromCloud
};


struct MocapOptions {
	bool useActiveBeacon = false;
	MocapMode mode = MocapPassthrough;
};

/**
 * Used to interface pose feedback from motion capture systems with the vehicles
 */
class Mocap {
public:
	Mocap(const MocapOptions &opts);
	~Mocap();

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
	 * @param markers optional list of reference marker positions on the vehicle. these will be used to correct the position and orientation to the reference center-of-mass frame
	 */
	void track(Vehicle *v, int id, const vector<Vector3d> &markers = {});


	void start_recording();
	void stop_recording();
	void resync();

private:

	void onNatNetFrame(const optitrack::NatNetFrame *frame);

	map<int, Vehicle *> tracked;
	optitrack::NatNetClient *client;

	RigidBodyTracker *tracker;

	MocapOptions options;

	Time lastBeaconPing; /**< If we are using the beacon for tracking, this is the last time we tried to turn it on */

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
	unsigned id; /**< The id of a rigid body or other entity */

	bool fixed; // Whether or not the id should be allowed to change

};

// TODO: Have a different object distinguish the rigid bodies which would internally reference many inner markers each with visibility data
/**
 *
 */
struct MocapTrackedMarker {
	Vector3d position;
	Vector3d velocity;
	Time firstSeen;
	Time lastSeen;
	bool visible; /**< Whether or not it will visible in the most recent frame */

	// TODO: This is not currently being used
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


/**
 * An exact formulation of ICP for matching rigid body points forward. This does not use any ANN library as this will usually be using only a few points per body
 *
 * @return whether or not it suceeded reasonably
 */
bool iterative_closest_point(const vector<Vector3d> &query, const vector<Vector3d> &pts, Matrix3d *R, Vector3d *t, vector<int> *indices);


}

#endif
