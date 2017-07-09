#include "calibration.h"

#include <ceres/ceres.h>


using namespace std;



struct DistanceFunctor {

	DistanceFunctor(double measured_distance) : measured_distance(measured_distance) {}

	template <typename T>
	bool operator()(const T* const a, const T* const b, T* residual) const {

		T dx = a[0] - b[0],
			dy = a[1] - b[1],
			dz = a[2] - b[2];

		T predicted_distance = sqrt(dx*dx + dy*dy + dz*dz);

		residual[0] = T(measured_distance) - predicted_distance;

		return true;
	}

	double measured_distance;

};



namespace tansa {


bool BeaconCalibration::estimate_positions(const MatrixXd &distances, vector<Vector3d> *out) {

	unsigned N = distances.rows();

	// Estimates of point locations
	// Initialize to random values for now
	out->resize(N);



	ceres::Problem problem;

	double maxDistance = 0;

	for(unsigned i = 0; i < N; i++) {
		for(unsigned j = i + 1; j < N; j++) {

			if(distances(i, j) > maxDistance) {
				maxDistance = distances(i, j);
			}

			DistanceFunctor *df = new DistanceFunctor(distances(i, j));

			ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<DistanceFunctor, 1, 3, 3>(df);
			problem.AddResidualBlock(cost_function, NULL, &((*out)[i](0)), &((*out)[j](0)));
		}
	}

	// Initialize to random values for now scaled by the size of the area
	for(unsigned i = 0; i < N; i++) {
		(*out)[i] = maxDistance*Vector3d::Random();
	}


	ceres::Solver::Options options;
	//options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";
	//cout << " -> " << x << "\n";

	return summary.termination_type == ceres::CONVERGENCE;
}


}
