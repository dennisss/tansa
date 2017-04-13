#include "gtest/gtest.h"

#include <tansa/mocap.h>

#include <iostream>

using namespace std;
using namespace tansa;


TEST(MocapTrackerTest, ICP) {
	/*
	// TODO: Fix this

	// Generate random point cloud
	vector<Vector3d> a;

	a.push_back(Vector3d(1, 1, 0));
	a.push_back(Vector3d(-1, 1, 0));
	a.push_back(Vector3d(-1, -1, 0.2));
	a.push_back(Vector3d(1, -2, 0));


	Matrix3d R = (AngleAxisd(0.3, Vector3d::UnitZ()) * AngleAxisd(0.1, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitZ())).matrix();

	Vector3d t = Vector3d(0.1, 0.05, 0.01);

	vector<Vector3d> b;
	for(int i = 0; i < a.size(); i++) {
		b.push_back(R*a[i] + t);
	}

	// Provide an outlier which would get matched first to the
	b.push_back(Vector3d(1, 1, 0));


	Matrix3d R_out = Matrix3d::Identity();
	Vector3d t_out = Vector3d::Zero();
	vector<int> indices;

	iterative_closest_point(b, a, &R_out, &t_out, &indices);

	ASSERT_TRUE(R_out.isApprox(R, 0.001));
	ASSERT_TRUE(t_out.isApprox(t, 0.001));

	*/
}
