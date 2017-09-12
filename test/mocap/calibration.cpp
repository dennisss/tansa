#include "gtest/gtest.h"

#include <tansa/mocap.h>
#include <tansa/vision/solve_pnp.h>

#include <iostream>

using namespace std;
using namespace tansa;


TEST(MocapCalibrationTest, findWandProjection) {


	vector<Vector3d> pattern = {
		{ -0.125, 0, 0 },
		{ 0, 0, 0 },
		{ 0.250, 0, 0 },
		{ 0, 0.2, 0 }
	};


	CameraModel cam = CameraModel::Default(0);

	vector<Vector2d> observed;
	for(int i = 0; i < pattern.size(); i++) {
		observed.push_back(cam.projectPoint(pattern[i]));
		cout << observed[i] << endl;
	}

	solvePnP(pattern, observed, &cam);


}
