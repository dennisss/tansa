#include <tansa/vision/solve_pnp.h>

#ifdef USE_CERES
#include <ceres/ceres.h>
#include "reprojection_error.h"
#endif

#include <iostream>

using namespace std;

// Makes a number from -1 to 1
#define RANDOM (-1 + static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(2))))

namespace tansa {


bool solvePnP(const vector<Vector3d> &points, const vector<Vector2d> &observed, CameraModel *cam, bool useInitial) {
#ifdef USE_CERES
	ceres::Problem problem;

	// Camera parameters held constant
	double instrinsics[3];
	instrinsics[0] = cam->fx;
	instrinsics[1] = cam->k1;
	instrinsics[2] = cam->k2;

	// Output camera extrinsic parameters that we are optimizing
	double extrinsics[6];

	if(useInitial) {
		// Initialize from what we were given
		Vector3d Rinit = toRodrigues(cam->rotation.transpose());
		Vector3d tinit = -1*cam->rotation.transpose()*cam->position;
		extrinsics[0] = Rinit.x(); extrinsics[1] = Rinit.y(); extrinsics[2] = Rinit.z();
		extrinsics[3] = tinit.x(); extrinsics[4] = tinit.y(); extrinsics[5] = tinit.z();
	}
	else {
		// Try with a random guess
		for(int i = 0; i < 6; i++) {
			extrinsics[i] = 2*RANDOM;
		}
	}

	// We have one block per point
	for(int i = 0; i < points.size(); i++) {

		double *point = (double *) &(points[i](0));

		ReprojectionError *f = new ReprojectionError(observed[i].x(), observed[i].y(), cam->cx, cam->cy);

		ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 6, 3>(f);
		problem.AddResidualBlock(cost_function, NULL, instrinsics, extrinsics, point);
		problem.SetParameterBlockConstant(point);
		problem.SetParameterBlockConstant(instrinsics);
	}


	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";


	bool success = summary.termination_type == ceres::CONVERGENCE;

	if(success) {
		// Unpack
		Matrix3d R = fromRodrigues(Vector3d(extrinsics[0], extrinsics[1], extrinsics[2]));
		Vector3d t(extrinsics[3], extrinsics[4], extrinsics[5]);

		// Transform back
		cam->rotation = R.transpose();
		cam->position = -R.transpose()*t;
	}

	return success;

#else
	cout << "solvePnP requires Ceres solver" << endl;
	return false;
#endif
}


}
