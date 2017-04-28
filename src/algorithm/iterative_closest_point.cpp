#include <tansa/algorithm.h>

using namespace std;

namespace tansa {

// This should work by exactly grabbing points around a velocity weighted gaussian centroid around the current estimate. Although, due to the fact that we forward estimate the points based on the current estiamte, this is less important
// TODO: Use an NN library / kD trees
// TODO: Do not immediately modidy the output varaibles
bool IterativeClosestPoint::align(const vector<Vector3d> &query, const vector<Vector3d> &pts, Matrix3d *R, Vector3d *t, vector<int> *indices) {
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

	for(int it = 0; it < options.max_iterations; it++) { // refinement iterations

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
					if(d > options.max_distance)
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
			// Have all the weights based on distance
			if(idx == -1 || (pts[idx] - query_tf[i]).norm() > options.max_distance) {
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
	if(e > options.max_error) {
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

}
