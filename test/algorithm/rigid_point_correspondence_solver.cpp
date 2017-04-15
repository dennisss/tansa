#include "gtest/gtest.h"

#define RANDFLOAT ((2.0 * static_cast<double>(rand()) / static_cast <double>(RAND_MAX)) - 1.0)

#include <tansa/algorithm.h>

using namespace tansa;
using namespace std;

TEST(Algorithms, RigidPointCorrespondenceSolverIdeal) {

	vector<Vector3d> as;
	for(int i = 0; i < 5; i++) {
		as.push_back(Vector3d(RANDFLOAT, RANDFLOAT, RANDFLOAT));
	}

	// Make a second set of vectors that is shuffled
	vector<Vector3d> bs = as;
	bs[0] = as[1];
	bs[1] = as[0];

	// Generate a random rotation from euler angles
	Vector3d eR = Vector3d::Random();
	Matrix3d R = (AngleAxisd(eR[0], Vector3d::UnitZ()) * AngleAxisd(eR[1], Vector3d::UnitX()) * AngleAxisd(eR[2], Vector3d::UnitZ())).matrix();

	// Random translation
	Vector3d t = Vector3d::Random();

	// Transform second set and add some noise
	for(int i = 0; i < 5; i++) {
		Vector3d n = Vector3d(RANDFLOAT, RANDFLOAT, RANDFLOAT) / 100; // Add ~1% error
		bs[i] = R * bs[i] + t + n;
	}


	RigidPointCorrespondenceSolver s;

	// Determine correspondence
	vector<int> order_out;
	s.solve(as, bs, &order_out, true);
	// TODO: Assert

	// Unshuffle according to order
	vector<Vector3d> bs_out;
	s.arrange(bs, order_out, &bs_out);

	// Find the transform
	Matrix3d R_out; Vector3d t_out;
	rigid_transform_solve(as, bs_out, R_out, t_out);

	ASSERT_TRUE(R_out.isApprox(R, 0.04));
	ASSERT_TRUE(t_out.isApprox(t, 0.04));

}
