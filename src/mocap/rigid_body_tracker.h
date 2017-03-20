#ifndef TANSA_MOCAP_RIGID_BODY_TRACKER_H_
#define TANSA_MOCAP_RIGID_BODY_TRACKER_H_

#include <tansa/vehicle.h>


#define PHASE_IDLE 0
#define PHASE_MASK 1
#define PHASE_SEARCH 2
#define PHASE_EXCLUDE 3
#define PHASE_FINALIZE 4


namespace tansa {

/**
 * A Physical entity with markers fixed w.r.t a center of mass
 */
struct RigidBody {
	unsigned id; /**< Unique identifier that was never assigned to another rigid body */
	Vector3d position;
	Vector3d velocity;

	Quaterniond orientation;
	Vector3d angularVelocity;

	Time firstSeen, lastSeen;

	// TODO: Include data as to whether or not it was just seen


	int nframes;

	vector<Vector3d> markers; /**< List of one or more markers defined in the body frame */
};

// For storing data
struct RegistrationProcessData {
	int phase = PHASE_IDLE;

	int activeVehicle = 0; /**< Index of the current vehicle we are trying to find */

	Time start; /**< When the process started */

	Time beaconOn; /**< When the beacon was turned on */

	unsigned maskId; /**< Marker must have an id greater than this  */

	vector<unsigned> candidates;

};

/**
 * Extracts rigid bodies from a point cloud
 */
class RigidBodyTracker {
public:

	RigidBodyTracker();


	void track(Vehicle *v);


	// TODO: Derive timestamp from OurTime -  Motive reported latency - NatNet ping time
	/**
	*
	*/
	void update(const vector<Vector3d> &cloud, const Time &t);

	void update_registration(const Time &t);

private:

	vector<Vector3d> model;

	unsigned lastId = 0;

	// TODO: Make this a linked list
	vector<RigidBody> bodies;

	Time time;

	// Map of rigid body ids to vehicle indices
	map<unsigned, unsigned> registrations;

	RegistrationProcessData data;


	vector<Vehicle *> vehicles;




};


}


#endif
