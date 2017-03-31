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


/**
 * Used to interface pose feedback from motion capture systems with the vehicles
 */
class Mocap {
public:
	Mocap(MocapMode m = MocapPassthrough);
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

private:

	void onNatNetFrame(const optitrack::NatNetFrame *frame);

	map<int, Vehicle *> tracked;
	optitrack::NatNetClient *client;

	RigidBodyTracker *tracker;


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

struct MocapTrackedMarker {
	Vector3d position;
	Vector3d velocity;
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


/**
 * Given two sets of unlabeled points, this will compute the transform that best matches them,
 * So: b_j = M * a_i
 *
 * Based on approach in 'Determining Correspondences and Rigid Motion of 3-D Point Sets with Missing Data' by Wang et al.
 * This ideal version assumes that there are no missing or outlier points
 *
 * @param as first set of points
 * @param bs second set of points
 * @param c the correspondences. such that bs[c[i]] = as[i]
 */
void correspondence_solve_ideal(const vector<Vector3d> &as, const vector<Vector3d> &bs, vector<unsigned> *c);

/**
 * Rearranges the 'bs' set from correspondence_solve_ideal to match the ordering of the points in 'as'
 */
void correspondence_arrange(const vector<Vector3d> &bs, const vector<unsigned> &c, vector<Vector3d> *out);

/**
 * Pretty standard SVD based recovery of labeled point set rigid transformation recovery
 * Computes bs[i] = R as[i] + t
 */
void rigid_transform_solve(const vector<Vector3d> &as, const vector<Vector3d> &bs, Matrix3d &R, Vector3d &t, const vector<double> &w = {});

/**
 *
 *
 * @return whether or not it suceeded reasonably
 */
bool iterative_closest_point(const vector<Vector3d> &query, const vector<Vector3d> &pts, Matrix3d *R, Vector3d *t, vector<int> *indices);


}

#endif
