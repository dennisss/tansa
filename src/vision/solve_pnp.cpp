#include <tansa/vision/solve_pnp.h>

#include <ceres/ceres.h>
#include "reprojection_error.h"

#include <iostream>

using namespace std;


namespace tansa {


double solvePnP(const vector<Vector3d> &points, const vector<Vector2d> &observed, CameraModel *cam, bool useInitial) {

	ceres::Problem problem;

	// Output camera extrinsic parameters that we are optimizing
	double camera[6];

	if(useInitial) {
		// Initialize from what we were given
		Vector3d Rinit = toRodrigues(cam->rotation.transpose());
		Vector3d tinit = -1*cam->rotation.transpose()*cam->position;
		camera[0] = Rinit.x(); camera[1] = Rinit.y(); camera[2] = Rinit.z();
		camera[3] = tinit.x(); camera[4] = tinit.y(); camera[5] = tinit.z();
	}
	else {
		// Try to smart a smart random guess
		// TODO: possibly perform alignment based on eigenvector alignment
		// Also, we know that obviously, all points should be in +z w.r.t the camera
		for(int i = 0; i < 6; i++) {
			camera[i] = rand();
		}
	}

	// We have one block per point
	for(int i = 0; i < points.size(); i++) {

		double *point = (double *) &(points[i](0));

		// TODO: Instead we should use the same model as the bundle adjustment but allow different parameter blocks to remain constant
		ReprojectionError *f = new ReprojectionError(*cam, observed[i]);

		ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<SolvePnPReprojectionError, 2, 6, 3>(f);
		problem.AddResidualBlock(cost_function, NULL, camera, point);
		problem.SetParameterBlockConstant(point);
		cout << points[i].transpose() << endl;
	}


	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";
	//cout << " -> " << x << "\n";

	cout << "INIT: " << Rinit.transpose() << "      " << tinit.transpose() << endl;
	cout << "FINAL:" << camera[0] << " " << camera[1] << " " << camera[2] << " " << camera[3] << " " << camera[4] << " " << camera[5] << endl;

	cout << cam->position.transpose() << endl;

	// Unpack back into the

	return 0;

	// return summary.termination_type == ceres::CONVERGENCE;


}










}
