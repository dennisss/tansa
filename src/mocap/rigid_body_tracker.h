#ifndef TANSA_MOCAP_RIGID_BODY_TRACKER_H_
#define TANSA_MOCAP_RIGID_BODY_TRACKER_H_

#include <tansa/vehicle.h>


#define PHASE_IDLE 0
#define PHASE_MASK 1
#define PHASE_SEARCH 2
#define PHASE_INCLUDE 3
#define PHASE_EXCLUDE 4
#define PHASE_FINALIZE 5


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
	bool visible;

	int nframes;

	vector<Vector3d> markers; /**< List of one or more markers defined in the body frame */
};


/**
 *
 */
struct RigidBodyDescription {

	vector<Vector3d> referenceMarkers;


	double boundingRadius; /**< Precomputed radius of a sphere that could contain all points */


};

// For storing data
struct RegistrationProcessData {
	int phase = PHASE_IDLE;

	int activeVehicle = 0; /**< Index of the current vehicle we are trying to find */

	Time start; /**< When the process started */

	Time beaconToggle; /**< When the beacon was turned on */

	unsigned iteration; /** How many on-off cycles of the beacon have occured */

	unsigned maskId; /**< Marker must have an id greater than this  */

	vector<unsigned> candidates; /**< Ids of the markers we currently think may be beacons */


};

/**
 *
 */
struct RigidBodyTrackerSettings {
	double timeout = 0;
};

/**
 * Extracts rigid bodies from a point cloud
 */
class RigidBodyTracker {
public:

	RigidBodyTracker(const RigidBodyTrackerSettings &settings = RigidBodyTrackerSettings());


	void track(Vehicle *v);


	// TODO: Currently we assume that this is called at least 100 times a second regardless of network connectivity
	// TODO: Derive timestamp from OurTime -  Motive reported latency - NatNet ping time
	/**
	*
	*/
	void update(const vector<Vector3d> &cloud, const Time &t);

	void update_active_registration(const Time &t);

private:

	vector<Vector3d> model;

	unsigned lastId = 0;

	// TODO: Make this a linked list
	vector<RigidBody> bodies;

	Time time;

	// Map of vehicle indices to rigid body ids
	map<unsigned, unsigned> registrations;

	//
	map<unsigned, unsigned> registrationInv;

	RegistrationProcessData data;


	vector<Vehicle *> vehicles;




};


}


#endif
