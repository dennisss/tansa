#include <tansa/mocap.h>
#include <tansa/algorithm.h>
#include "rigid_body_tracker.h"

#include <iostream>
#include <map>
#include <algorithm>

using namespace std;

// TODO: Adaptively measure motion capture standard deviation and use this to generate kalman filter gaussians to filter which points should be matched against next.
	// Rather than matching against distance, we should match against probability

namespace tansa {

#define MAX_DEVIATION 0.12 // By default cap deviation at 80mm
#define ICP_MAX_ERROR 0.02
#define TIMEOUT 0.2 // Time in seconds after which to abandon a tracked entity

// TODO: If tracking is lost for half a second, enter landing mode
// If we see that two rigid bodies have collided, kill
/*
	Given,
	- Vehicles array (with active ir beacons)
	- All markers in the current frame

	It should
	- 1 to n point tracking
	- Be able to provide 3d thru 6d pose incremental pose feedback

	Maintain 'virtual' rigid bodies for every single entity in the point cloud
		- Finding the beacon defined as finding a rigid body of a single dot which no longer exists

	Initialization
	- Find flashing beacon
	- Perform correspondence match with model (and possible also pre-recorded pattern if re-initializing)
		- Enforce maximum deviation

	Update
	- Propagate/predict forward via linear and angular velocity
	- Refine using ICP
	- Double check result via model fitting

*/

// Finds the index of the closest point
int closest_point(const vector<Vector3d> &pts, Vector3d query, double *dist = NULL) {

	double mindist = 999999;
	int mini = -1;

	for(int i = 0; i < pts.size(); i++) {
		double d = (query - pts[i]).norm();
		if(d < mindist) {
			mindist = d;
			mini = i;
		}
	}

	if(dist != NULL)
		*dist = mindist;

	return mini;
}

// This should work by exactly grabbing points around a velocity weighted gaussian centroid around the current estimate. Although, due to the fact that we forward estimate the points based on the current estiamte, this is less important
// TODO: Use an NN library / kD trees
// TODO: Do not immediately modidy the output varaibles
bool iterative_closest_point(const vector<Vector3d> &query, const vector<Vector3d> &pts, Matrix3d *R, Vector3d *t, vector<int> *indices) {
	/*
		Find closest points to each other point,
		- Don't allow two points to have the same closest point
		- Weight matches based on distance (outliers shouldn't impact the match)
	*/

	Matrix3d Rinit = *R;
	Vector3d tini = *t;

	indices->resize(query.size());

	vector<Vector3d> query_tf; query_tf.resize(query.size());

	vector<Vector3d> pts_subset; pts_subset.resize(query.size());
	vector<double> pts_w; pts_w.resize(query.size(), 1);

	for(int it = 0; it < 3; it++) { // refinement iterations

		// Generate current query set
		for(int i = 0; i < query.size(); i++)
			query_tf[i] = (*R) * query[i] + (*t);

		// Find closest points
		// TODO: Fast reject all points not close to the centroid : Represent query as a centroid + distance
		for(int i = 0; i < query.size(); i++) { // Reset indices
			(*indices)[i] = -1;
		}


		// Compute the distance of each query point to it's top N other points

		// Mapping a pts index to a query point index
		map<int, int> matches;
		matches.clear();

		// TODO: Use the assignment problem solver
		int it2;
		for(it2 = 0; it2 < query.size(); it2++) {

			bool changed = false;
			for(int i = 0; i < query.size(); i++) {

				if((*indices)[i] != -1) // Already matched, do nothing
					continue;


				int bestJ = -1;
				double bestDist = 0;

				// Find a match for this point
				for(int j = 0; j < pts.size(); j++) {
					double d = (pts[j] - query_tf[i]).norm();

					// Reject right away any points very far away from the
					if(d > 0.4)
						continue;

					// Reject if not better than our current option
					if(!(bestJ == -1 || d < bestDist)) {
						continue;
					}

					// Already used better by another query point
					if(matches.count(j) == 1) {
						if((pts[j] - query_tf[matches[j]]).norm() < d) {
							continue;
						}
					}

					bestJ = j;
					bestDist = d;
				}

				if(bestJ == -1)
					continue;


				// Kick out the other match
				if(matches.count(bestJ) == 1) {
					(*indices)[matches[bestJ]] = -1;
				}

				(*indices)[i] = bestJ;
				matches[bestJ] = i;
				changed = true;
			}

			if(!changed)
				break;
		}

		if(it2 == query.size()) {
			cout << "ICP: Terminated early" << endl;
			return false;
		}


		// TODO: If this isn't the first iteration and the indices haven't changed, we can terminate the iterations

		// Extract pts subset and weights: rejecting indices that result in large discontuities
		int n = 0;
		for(int i = 0; i < indices->size(); i++) {
			int idx = (*indices)[i];

			// Outlier rejection
			// TODO: Use an adaptive threshold
			if(idx == -1 || (pts[idx] - query_tf[i]).norm() > 0.4) {
				cout << "ICP: Reject point!" << endl;
				pts_w[i] = 0;
			}
			else {
				Vector3d p = pts[idx];
				pts_subset[i] = p;
				pts_w[i] = 1.0;
				n++;
			}
		}

		if(n < 3) {
			cout << "ICP: Not enough correspondences. Only have: " << n << endl;
			/*
			for(int i = 0; i < indices->size(); i++) {
				cout << (*indices)[i] << endl;

			}
			cout << "Pts" << endl;
			for(int i = 0; i < pts.size(); i++) {
				cout << pts[i].transpose() << endl;
			}

			cout << "Query" << endl;
			for(int i = 0; i < query.size(); i++) {
				cout << query_tf[i].transpose() << endl;
			}

			cout << "init" << endl;
			cout << Rinit << endl;
			cout << tini << endl;
			*/
			return false;
		}


		// Recover transformation
		Matrix3d dR;
		Vector3d dt;
		rigid_transform_solve(query_tf, pts_subset, dR, dt, pts_w);

		// Apply
		*R = dR * (*R);
		*t = (*t) + dt;
	}


	// Generate a final matching score
	double e = 0.0;
	for(int i = 0; i < query.size(); i++) {
		e += (pts[(*indices)[i]] - ((*R) * query[i] + (*t))).squaredNorm();
	}
	e = sqrt(e);

	// This error value can be even smaller as we can assume that they are rigidly attached
	// (use some value based on the standard deviation of individual marker deviations)
		// we should be able to compute the likelihood of the two sets to be matching
	if(e > ICP_MAX_ERROR) {
		cout << "ICP: Error too big: " << e << endl;

		for(int i = 0; i < indices->size(); i++) {
			cout << (*indices)[i] << endl;

		}
		cout << "Pts" << endl;
		for(int i = 0; i < pts.size(); i++) {
			cout << pts[i].transpose() << endl;
		}

		cout << "Query" << endl;
		for(int i = 0; i < query.size(); i++) {
			cout << query_tf[i].transpose() << endl;
		}

		cout << "init" << endl;
		cout << Rinit << endl;
		cout << tini << endl;

		return false;
	}

	return true;
}


RigidBodyTracker::RigidBodyTracker(const RigidBodyTrackerSettings &s) {

	json j = DataObject::LoadFile("config/models/x260/index.js");

	model.resize(0);
	for(int i = 0; i < j["markers"].size(); i++) {
		json p = j["markers"][i]["position"];
		model.push_back(Vector3d(p[0], p[1], p[2]));
	}

	// TODO: This is mainly for testing single marker
	model.resize(1);
}

void RigidBodyTracker::track(Vehicle *v) {
	vehicles.push_back(v);
}

// TODO: Compute the standard deviation somewhere to know the whole system error

// This will internally propagate forward the
void RigidBodyTracker::update(const vector<Vector3d> &cloud, const Time &t) {

	vector<Vector3d> pts = cloud;

	double dt = t.since(this->time).seconds();


	// TODO: Maybe prioritize matching rigid bodies with more than 1 point
	// TODO: We may need to do a few iterations of this if there are collisions in nearest points
	for(int i = 0; i < bodies.size(); i++) {

		RigidBody &b = bodies[i];

		// Remove if it hasn't been seen in a while
		if(t.since(b.lastSeen).seconds() > TIMEOUT) {

			if(registrationInv.count(b.id) == 1) {
				unsigned v = registrationInv[b.id];
				registrationInv.erase(b.id);
				registrations.erase(v);
			}


			bodies.erase(bodies.begin() + i);
			i--;
			continue;
		}

		Vector3d p_next = b.position + dt * b.velocity;
		Quaterniond o_next = b.orientation; // TODO: Integrate this too

		bool seen = false;
		// TODO: What if multiple rigid bodies have the same closest point
		if(b.markers.size() == 1) { // Simple case
			double dist;
			int ptI = closest_point(pts, p_next, &dist);
			if(ptI >= 0 && dist < MAX_DEVIATION) {
				p_next = pts[ptI];
				pts.erase(pts.begin() + ptI);
				seen = true;
			}
		}
		else {
			// For finding rigid bodies once they disappeared, apply mean-shift clustering using a radius equivalent to that of the query. if we don't get a match, add more means to the search until all points in the cloud are accounted for

			vector<int> indices;

			Matrix3d Ri = o_next.toRotationMatrix();
			Vector3d ti = p_next;

			// TODO: If ICP fails we should have special behavior when < 3 markers are visible
			// -> For one dot, we are essentially reducing to a single marker tracking problem
			// -> With only two markers, it becomes a Quaternion::FromTwoVectors problem
			/*
				If we have 2 markers, then we can determine auto-gyro bias estimation

				- We record initial yaw orientation y0 with mocap orientation ym0 at time 0
				- Later we record any set: yt and ymt at time 't'
				- If the bias was perfect, then yt - y0 == ymt - ym0
					- (yt - y0) - (ymt - ym0) = ydiff is the unexplained rotation angle integrated from the gyro
					- differentiating, we get a bias of (ydiff / t) = b
					- we will use gradient descent to incrementally add to the current gyro bias until
			*/

			if(iterative_closest_point(b.markers, pts, &Ri, &ti, &indices)) {

				o_next = Quaterniond(Ri);
				p_next = ti;

				std::sort(indices.begin(), indices.end());
				for(int j = indices.size() - 1; j >= 0; j--) {
					if(indices[j] >= 0)
						pts.erase(pts.begin() + indices[j]);
				}

				//cout << "Rigid Body Track: " << p_next.transpose() << " : " << b.nframes << endl;

				seen = true;

			}

		}


		Vector3d v_next = (p_next - b.position) / dt;
		double alpha = 0.8;
		b.velocity = alpha*b.velocity + (1.0 - alpha)*v_next;
		b.position = p_next;
		b.orientation = o_next;
		b.visible = seen;

		if(seen) {
			b.lastSeen = t;
			b.nframes++;

			// Single dot case: we don't directly have orientation
			if(registrationInv.count(b.id) == 1) {

				Vehicle *v = vehicles[registrationInv[b.id]];

				Quaterniond orient;
				if(b.markers.size() == 1) {

					if(v->flying) {
						orient = Quaterniond(2, 2, 2, 2); // An invalid orientation to indicate that we have no orientation data
					}
					else { // On the ground, we assume that the drone was placed facing forwards
						float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
						r /= 100.0;
						orient = AngleAxisd(r, Vector3d::UnitZ());
					}

					v->mocap_update(b.position, orient, t);
				}
				else {
					v->mocap_update(b.position, b.orientation, t);
				}
			}
		}

	}


	// Create new bodies for all untrackable points
	for(Vector3d p : pts) {
		// TODO: Standardize this set of commands
		RigidBody b;
		b.id = ++lastId;
		b.position = p;
		b.velocity = Vector3d(0, 0, 0);
		b.nframes = 1;
		b.orientation = Quaterniond(1, 0, 0, 0);
		b.angularVelocity = Vector3d(0, 0, 0);
		b.markers.push_back(Vector3d(0, 0, 0));
		b.firstSeen = t;
		b.lastSeen = t;
		b.visible = true;
		bodies.push_back(b);
	}


	this->update_active_registration(t);

	this->time = t;
}



// This handles the active-ir based registration of new rigid bodies
// There should also be a separate routine for performing passive detection of the rigid bodies
// - The passive routine would:
//    - Apply mean clustering around point sets
//    - Grab all points in a single region
//    - Try to do an exact correspondence match, otherwise go to the next mean

// TODO: This should under no condition be allowed to fail
void RigidBodyTracker::update_active_registration(const Time &t) {

	int &phase = data.phase;

	// If we aren't doing anything, see if any vehicles can be initialized
	if(phase == PHASE_IDLE) {

		// Find the next vehicle to try and lock on to
		int nextI = -1;
		for(int i = 0; i < vehicles.size(); i++) {
			int ii = (data.activeVehicle + i + 1) % vehicles.size();
			if(registrations.count(ii) == 0 && vehicles[ii]->connected == true) {
				nextI = ii;
				break;
			}
		}

		if(nextI == -1)
			return;

		data.start = t;
		data.phase = PHASE_MASK;
		data.activeVehicle = nextI;
		vehicles[nextI]->set_beacon(false);
	}


	double elapsed = t.since(data.start).seconds();

	Vehicle *v = vehicles[data.activeVehicle];

	// Essentially, we need to oscillate the value of the beacon to ensure that we definately choose the correct beacon
	// Ideally I'd like to not use an FFT based method and instead just count the pulse width with a highpass filter and perform a few sequences of that
	// Transitions

	// Wait for the beacon to turn off and then mask all existing markers
	if(phase == PHASE_MASK && elapsed >= 0.2) { // Allowed time per phase <- This must be smaller than the time it takes for a track to be abandoned
		data.maskId = lastId;
		phase = PHASE_SEARCH;
		v->set_beacon(true);
		data.beaconToggle = t;
		data.iteration = 0;
	}
	else if(phase == PHASE_SEARCH && elapsed >= 0.1) { // We should also

		// TODO: Also verify the pulse width of the dot, we should maintain a record and

		data.candidates.resize(0);
		cout << bodies.size() << endl;
		for(int i = 0; i < bodies.size(); i++) {
			if(bodies[i].id > data.maskId) { // TODO: Also verify that it is currently visible
				double lat = bodies[i].firstSeen.since(data.beaconToggle).seconds() * 1000;

				if(lat > 70) {
					cout << "Reject Latency: " << lat << endl;
					continue;
				}

				// TODO: USe real times
				cout << "Beacon Latency: " << lat << "ms" << endl;

				cout << bodies[i].position.transpose() << endl;

				data.candidates.push_back(bodies[i].id);


				// TODO: If greater than 50ms, then probably not the beacon

				//cout << bodies[i].position.transpose() << endl;
				//cout << bodies[i].id << endl;
			}
		}

		cout << "Found " << data.candidates.size() << endl;

		/*
		if(data.candidates.size() == 1) {
			phase = PHASE_FINALIZE;
		}
		else {
		*/

		phase = PHASE_EXCLUDE;
		v->set_beacon(false);
		data.beaconToggle = t;

		/*
		}
		*/
	}
	else if(phase == PHASE_INCLUDE && elapsed >= 0.1) { // After a period of having the beacon on
		if(data.iteration >= 4) {
			data.phase = PHASE_FINALIZE;
			return;
		}

		bool good = false;
		for(int ci = 0; ci < data.candidates.size(); ci++) {
			unsigned c = data.candidates[ci];

			// Verify that the point
			for(int i = 0; i < bodies.size(); i++) {
				if(bodies[i].id == c) {
					double lat = bodies[i].firstSeen.since(data.beaconToggle).seconds() * 1000;

					if(lat > 70) {
						cout << "Reject Latency: " << lat << endl;
						break;
					}

					good = true;
					break;
				}
			}

			if(!good) {
				data.candidates.erase(data.candidates.begin() + ci);
				ci--;
			}
		}

		data.phase = PHASE_EXCLUDE;
		data.beaconToggle = t;
		v->set_beacon(false);

	}
	else if(phase == PHASE_EXCLUDE && elapsed >= 0.1) { // After a period of having the beacon off

		// TODO: This is essentially the same thing as the INCLUDE phase except firstSeen is switched to lastSeen

		bool good = false;
		for(int ci = 0; ci < data.candidates.size(); ci++) {
			unsigned c = data.candidates[ci];

			// Verify that the point
			// TODO: Reject using points assigned to other bodies if they are single marker bodies
			for(int i = 0; i < bodies.size(); i++) {
				if(bodies[i].id == c) {
					double lat = bodies[i].lastSeen.since(data.beaconToggle).seconds() * 1000;

					if(lat > 70) {
						cout << "Reject Latency: " << lat << endl;
						break;
					}

					good = true;
					break;
				}
			}

			if(!good) {
				data.candidates.erase(data.candidates.begin() + ci);
				ci--;
			}
		}


		data.phase = PHASE_INCLUDE;
		data.beaconToggle = t;
		v->set_beacon(true);
		data.iteration++;
	}
	else if(phase == PHASE_FINALIZE) {

		// TODO: Try all candidates and see accept if only one is good
		if(data.candidates.size() != 1) {
			cout << "Don't have exactly one candidate: Instead have " << data.candidates.size() << endl;
			data.phase = PHASE_IDLE;
			return;
		}

		// Single dot case
		if(model.size() == 1) {
			registrations[data.activeVehicle] = data.candidates[0];
			registrationInv[data.candidates[0]] = data.activeVehicle;
			data.phase = PHASE_IDLE;
			return;
		}


		vector<Vector3d> pts;
		for(int i = 0; i < bodies.size(); i++) {
			if(bodies[i].id == data.candidates[0]) {
				pts.push_back(bodies[i].position);
				bodies.erase(bodies.begin() + i);
				break;
			}
		}

		if(pts.size() == 0) {
			cout << "Lost the candidate" << endl;
			data.phase = PHASE_IDLE;
			return;
		}

		// Get all local points
		for(int i = 0; i < bodies.size(); i++) {
			// TODO: Generalize this distance
			if((bodies[i].position - pts[0]).norm() < 0.2) {
				pts.push_back(bodies[i].position);
				cout << "pt: " << bodies[i].position << endl;
				bodies.erase(bodies.begin() + i);
				i--;
			}
		}

		cout << "Have: " << pts.size() << endl;

		if(pts.size() < model.size()) {
			cout << "Too few markers visible" << endl;
			data.phase = PHASE_IDLE;
			return;
		}


		// After this the rigid body estimator will require that:
		// - There are at least 4 correspondes
		// - Check strength of eigenvalues of scatter matrix to figure out if the shape is symmetric
		//    - All three eigenvalues should be roughly the same but not the same?
		// - Enforce maximum distance between a single correspondence
		// - Enforce overall norm-2 error max threshold

		RigidPointCorrespondenceSolver solver;

		vector<int> c;
		solver.solve(model, pts, &c); // TODO: Use outlier rejecting form to only select some points from the cloud
		if(c[0] != 0) {
			cout << "Mismatch with active beacon" << endl;
			// In this case fail
		}

		// TODO: Deal with any cases in which the points couldn't be matched
		// Rearrange points
		vector<Vector3d> corresponding;
		solver.arrange(pts, c, &corresponding);

		// Find initial pose
		Matrix3d R; Vector3d trans;
		// TODO: Weight non-active ir points higher
		rigid_transform_solve(model, corresponding, R, trans);

		// Inverse transform corresponding point cloud points back into reference frame
		for(int i = 0; i < corresponding.size(); i++) {
			corresponding[i] = R.transpose() * (corresponding[i] - trans);
		}

		// Compute matching error
		double e = 0;
		for(int i = 0; i < model.size(); i++) {
			e += (model[i] - corresponding[i]).squaredNorm();
		}
		e = sqrt(e);

		// TODO: Base this threshold on rigid body segment distances
		if(e > 0.02) {
			cout << "failed" << endl;
			data.phase = PHASE_IDLE;
			return;
		}


		RigidBody b;
		b.id = ++lastId;
		b.markers = corresponding;
		b.position = trans;
		b.velocity = Vector3d(0, 0, 0);
		b.orientation = Quaterniond(R);
		b.firstSeen = t;
		b.lastSeen = t;
		b.nframes = 1;
		b.visible = true;

		bodies.push_back(b);

		registrations[data.activeVehicle] = b.id;
		registrationInv[b.id] = data.activeVehicle;

		data.phase = PHASE_IDLE;
	}
	else { // Didn't transition
		return;
	}

	// Update start time of next phase
	data.start = t;

}



}
