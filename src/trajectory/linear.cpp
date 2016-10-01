#include <tansa/trajectory.h>

#include <vector>
#include <iostream>


using namespace std;

/*
	Linear trajectories go from point x1 @ t1 to point x2 @ t2 in the smoothest way possible
	The velocity and acceleration at the end points should be 0

	This constructor generates the optimal minimum jerk polynomial
*/

// Computes a derivative based on the 0th order evaluation
// So given [t^0, t^1, t^2, t^3, t^4 ...]
// Computes [0, 1*t^0, 2*t^1, 3*t^2, ...   ] for n = 1
VectorXd diffvec(const VectorXd t, int n) {

	VectorXd dt = VectorXd::Zero(t.size());

	for(int i = n; i < t.size(); i++) {
		float c = 1;
		for(int p = i; p > i - n; p--) {
			c *= p;
		}

		dt(i) = c * t(i - n);
	}

	return dt;
}

VectorXd powvec(double t, int n) {
	VectorXd v(n);
	v(0) = 1;
	for(int i = 1; i < n; i++) {
		v(i) = v(i - 1) * t;
	}

	return v;
}

LinearTrajectory::LinearTrajectory(Point x1, double t1, Point x2, double t2) {

	int n = 6; // number of coefficients to use

	// Vectorize the time powers
	VectorXd tvec1 = powvec(t1, n), tvec2 = powvec(t2, n);

	// Compute derivatives of time
	vector<VectorXd> dts1(n / 2), dts2(n / 2);
	dts1[0] = tvec1;
	dts2[0] = tvec2;

	// Differentiate
	for(int i = 1; i < dts1.size(); i++) {
		dts1[i] = diffvec(tvec1, i);
		dts2[i] = diffvec(tvec2, i);
	}


	// Setup 'A' in Ax = b : it is common for all axes
	MatrixXd A(n, n); // First three rows are position, velocity, acceleration of first point. Last three for
	for(int i = 0; i < dts1.size(); i++) {
		A.block(i, 0, 1, n) = dts1[i].transpose();
	}
	for(int i = 0; i < dts2.size(); i++) {
		A.block(i + dts1.size(), 0, 1, n) = dts2[i].transpose();
	}

	// cout << A << endl << endl;

	// Invert it once for speed
	MatrixXd Ainv = A.inverse();

	// cout << Ainv << endl << endl;

	// Compute for each axis
	for(int i = 0; i < 3; i++) {

		VectorXd b = VectorXd::Zero(n);

		// Constrain positions
		b(0) = x1(i);
		b(3) = x2(i);

		VectorXd x = Ainv*b;

		coeffs[i] = x;

		// cout << x << endl << endl;
	}
}



TrajectoryState LinearTrajectory::evaluate(double t) {

	VectorXd tv = powvec(t, coeffs[0].size());

	// Time vectors and velocity and acceleration
	VectorXd tv1 = diffvec(tv, 1), tv2 = diffvec(tv, 2);

	TrajectoryState s;
	// Compute each axis
	for(int i = 0; i < 3; i++) {
		s.position(i) = tv.dot(coeffs[i]);
		s.velocity(i) = tv1.dot(coeffs[i]);
		s.acceleration(i) = tv2.dot(coeffs[i]);
	}

	return s;
}
