#include <tansa/trajectory.h>

namespace tansa {


FourierTrajectory(const MatrixXd &A, const MatrixXd &B, double freq, double phase, Point center, double ts, double te) : Trajectory(ts, te) {

	if(A.rows() != 3 || B.rows() != 3) {
		throw "Must be 3d";
	}

	if(A.cols() != B.cols()) {
		throw "Mismatching length of series coefficients";
	}


	this->A = A;
	this->B = B;
	this->freq = freq;
	this->phase = phase;
	this->center = center;
}

TrajectoryState FourierTrajectory::evaluate(double t) {

	TrajectoryState s;
	s.position = center;
	s.velocity = Point::Zero();
	s.acceleration = Point::Zero();

	unsigned N = A.cols();
	for(unsigned k = 0; k < N; k++) {

		Vector3d &a_k = A.block<3, 1>(0, k),
				 &b_k = B.block<3, 1>(0, k);

		double theta = k*freq*t + k*phase;
		double dtheta = k*freq;
		double ddtheta = dtheta*dtheta;

		double cosTheta = cos(theta),
			   sinTheta = sin(theta);

		s.position += a_k * cosTheta + b_k * sinTheta;
		s.velocity += -a_k * dtheta * sinTheta + b_k * dtheta * cosTheta;
		s.acceleration += -a_k * ddtheta * cosTheta - b_k * ddtheta * sinTheta;
	}

	return s;
}



}
