#include "tansa/mocap.h"
#include "rigid_body_tracker.h"

#include <iostream>
#include <map>

using namespace std;


namespace tansa {

#define MAX_DEVIATION 0.02 // By default cap deviation at 20mm
#define TIMEOUT 0.2

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

// TODO: Use an NN library / kD trees
// TODO: Do not immediately modidy the output varaibles
/**
 * @return whether or not the ICP succeded reasonably
 */
bool iterative_closest_point(const vector<Vector3d> &pts, const vector<Vector3d> &query, Matrix3d *R, Vector3d *t, vector<int> *indices) {
	/*
		Find closest points to each other point,
		- Don't allow two points to have the same closest point
		- Weight matches based on distance (outliers shouldn't impact the match)
	*/

	indices->resize(query.size());

	vector<Vector3d> query_tf; query_tf.resize(query.size());

	vector<Vector3d> pts_subset; pts_subset.resize(query.size());
	vector<double> pts_w; pts_w.resize(query.size(), 1);

	for(int it = 0; it < 3; it++) {

		// Generate current query set
		for(int i = 0; i < query.size(); i++)
			query_tf[i] = (*R) * query[i] + (*t);

		// Find closest points
		// TODO: Fast reject all points not close to the centroid
		for(int i = 0; i < query.size(); i++) { // Reset indices
			(*indices)[i] = -1;
		}

		while(true) {
			bool changed = false;
			for(int i = 0; i < pts.size(); i++) {
				int bestj = -1;
				double bestdist = 0;
				for(int j = 0; j < query.size(); j++) {
					double dist = (pts[j] - query_tf[i]).norm();
					if((bestj == -1 || dist < bestdist)
						&& ((*indices)[j] == -1 || dist < (pts[(*indices)[j]] - query_tf[j]).norm())) { // To match with this point, we must be closer than the current option and
						bestdist = dist;
						bestj = j;
					}
				}

				if(bestj != -1) {
					(*indices)[bestj] = i;
					changed = true;
				}
			}

			if(!changed)
				break;
		}


		// Extract pts subset and weights: rejecting indices that result in large discontuities
		// TODO


		// Recover transformation
		Matrix3d dR;
		Vector3d dt;
		rigid_transform_solve(query_tf, pts_subset, dR, dt, pts_w);

		// Apply
		*R = dR * (*R);
		*t = (*t) + dt;


	}


	return true;
}


RigidBodyTracker::RigidBodyTracker() {

	json j = DataObject::LoadFile("config/models/x260.js");

	model.resize(0);
	for(int i = 0; i < j["markers"].size(); i++) {
		json p = j["markers"][i]["position"];
		model.push_back(Vector3d(p[0], p[1], p[2]));
	}

}

void RigidBodyTracker::track(Vehicle *v) {
	vehicles.push_back(v);
}

void RigidBodyTracker::update(const vector<Vector3d> &cloud, const Time &t) {

	vector<Vector3d> pts = cloud;

	double dt = t.since(this->time).seconds();


	// TODO: Maybe prioritize matching rigid bodies with more than 1 point
	// TODO: We may need to do a few iterations of this if there are collisions in nearest points
	for(int i = 0; i < bodies.size(); i++) {

		RigidBody &b = bodies[i];

		// Remove if it hasn't been seen in a while
		if(t.since(b.lastSeen).seconds() > TIMEOUT) {
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

			vector<int> indices;

			Matrix3d Ri = o_next.toRotationMatrix();
			Vector3d ti = p_next;

			if(iterative_closest_point(pts, b.markers, &Ri, &ti, &indices)) {

				o_next = Quaterniond(Ri);
				p_next = ti;

				for(int idx : indices) {

				}

				seen = true;

			}

		}


		b.velocity = (p_next - b.position) / dt;
		b.position = p_next;
		b.orientation = o_next;

		if(seen) {
			b.lastSeen = t;
			b.nframes++;
		}

	}


	// Create new bodies for all untrackable points
	for(Vector3d p : pts) {
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
		bodies.push_back(b);
	}


	this->update_registration(t);

	this->time = t;
}


void RigidBodyTracker::update_registration(const Time &t) {

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

	// Transitions
	if(elapsed >= 0.4) { // Allowed time per phase
		if(phase == PHASE_MASK) {
			data.maskId = lastId;
			phase = PHASE_SEARCH;
			v->set_beacon(true);
			data.beaconOn = t;
		}
		else if(phase == PHASE_SEARCH) {

			data.candidates.resize(0);
			for(int i = 0; i < bodies.size(); i++) {
				if(bodies[i].id > data.maskId) {
					data.candidates.push_back(bodies[i].id);

					cout << "Beacon Latency: " << (bodies[i].firstSeen.since(data.beaconOn).seconds() * 1000) << "ms" << endl;

					//cout << bodies[i].position.transpose() << endl;
					//cout << bodies[i].id << endl;
				}
			}

			cout << "Found " << data.candidates.size() << endl;

			// TODO: Only need to go into excluding mode if we have more than one candidate
			phase = PHASE_EXCLUDE;
			//vehicles[activeVehicle]->set_beacon(false);
		}
		else if(phase == PHASE_EXCLUDE) {
			phase = PHASE_FINALIZE;
		}
		else if(phase == PHASE_FINALIZE) {

			vector<Vector3d> pts;
			for(int i = 0; i < bodies.size(); i++) {
				if(bodies[i].id == data.candidates[0]) {
					pts.push_back(bodies[i].position);
					bodies.erase(bodies.begin() + i);
					break;
				}
			}

			// Get all local points
			for(int i = 0; i < bodies.size(); i++) {
				if((bodies[i].position - pts[0]).norm() < 0.2) {
					pts.push_back(bodies[i].position);
					cout << "pt: " << bodies[i].position << endl;
					bodies.erase(bodies.begin() + i);
					i--;
				}
			}

			cout << "Have: " << pts.size() << endl;

			if(pts.size() != model.size()) {
				cout << "Too many or too little markers visible" << endl;
			}


			vector<unsigned> c;
			correspondence_solve_ideal(model, pts, &c);
			if(c[0] != 0) {
				cout << "Mismatch with active beacon" << endl;
			}


			vector<Vector3d> model_corresponding;
			correspondence_arrange(model, model_corresponding, c);



			Matrix3d R; Vector3d tran;

			rigid_transform_solve(model_corresponding, pts, R, tran);


			cout << R << endl;

			RigidBody b;
			b.id = ++lastId;
			b.markers = model;
			b.position = tran;
			b.velocity = Vector3d(0, 0, 0);
			b.orientation = Quaterniond(R);
			b.firstSeen = t;
			b.lastSeen = t;
			b.nframes = 1;

			bodies.push_back(b);

			registrations[data.activeVehicle] = b.id;

			phase = PHASE_IDLE;
		}

		data.start = t;
	}

}



}
