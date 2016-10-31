#include <tansa/trajectory.h>
#include "utils.h"

namespace tansa {

PolynomialTrajectory::PolynomialTrajectory(const VectorXd c[], double t1, double t2)
 	: Trajectory(t1, t2) {

	for(int i = 0; i < PointDims; i++)
		coeffs[i] = c[i];
}

TrajectoryState PolynomialTrajectory::evaluate(double t) {

	VectorXd tv = powvec(t, coeffs[0].size());

	// Time vectors for velocity and acceleration
	VectorXd tv1 = diffvec(tv, 1), tv2 = diffvec(tv, 2);

	TrajectoryState s;
	// Compute each axis
	for(int i = 0; i < PointDims; i++) {
		s.position(i) = tv.dot(coeffs[i]);
		s.velocity(i) = tv1.dot(coeffs[i]);
		s.acceleration(i) = tv2.dot(coeffs[i]);
	}

	return s;
}

PolynomialTrajectory *PolynomialTrajectory::compute(const vector<Point> &c1, double t1, const vector<Point> &c2, double t2) {

	int n = 6; // number of coefficients to use

	// Vectorize the time powers
	VectorXd tvec1 = powvec(t1, n), tvec2 = powvec(t2, n);

	// Compute derivatives of time
	vector<VectorXd> dts1(n / 2), dts2(n / 2);
	dts1[0] = tvec1;
	dts2[0] = tvec2;

	// Differentiate
	for(unsigned i = 1; i < dts1.size(); i++) {
		dts1[i] = diffvec(tvec1, i);
		dts2[i] = diffvec(tvec2, i);
	}


	// Setup 'A' in Ax = b : it is common for all axes
	MatrixXd A(n, n); // First three rows are position, velocity, acceleration of first point. Last three for second point
	for(unsigned i = 0; i < dts1.size(); i++) {
		A.block(i, 0, 1, n) = dts1[i].transpose();
	}
	for(unsigned i = 0; i < dts2.size(); i++) {
		A.block(i + dts1.size(), 0, 1, n) = dts2[i].transpose();
	}

	// Invert it once for speed
	MatrixXd Ainv = A.inverse();

	VectorXd xs[PointDims];

	// Compute for each axis
	for(unsigned i = 0; i < PointDims; i++) {

		VectorXd b = VectorXd::Zero(n);

		// Adding in constraints

		// If they are now specified, we will set them them to 0 (else the system will be underconstrained)
		for(unsigned j = 0; j < c1.size(); j++) {
			b(j) = c1[j](i);
		}
		for(unsigned j = 0; j < c2.size(); j++) {
			b(j + dts1.size()) = c2[j](i);
		}

		VectorXd x = Ainv*b;
		xs[i] = x;
	}

	return new PolynomialTrajectory(xs, t1, t2);
}

}
