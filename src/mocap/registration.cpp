#include <vector>
#include <iostream>
#include <cfloat>

#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

/*
	Code for registering the pose of the vehicles based on known marker configurations and active IR beacons

*/

namespace tansa {


inline Vector3d centroid(const vector<Vector3d> &ps) {
	Vector3d c = Vector3d::Zero();
	for(unsigned i = 0; i < ps.size(); i++) {
		c = c + ps[i];
	}

	return c / ps.size();
}


void rigid_transform_solve(const vector<Vector3d> &as, const vector<Vector3d> &bs, Matrix3d &R, Vector3d &t) {

	/* Compute set centers */
	Vector3d cA = centroid(as), cB = centroid(bs);

	MatrixXd Ac(as.size(), 3), Bc(bs.size(), 3);
	for(unsigned i = 0; i < as.size(); i++){
		Ac.block<1,3>(i, 0) = as[i] - cA;
		Bc.block<1,3>(i, 0) = bs[i] - cB;
	}


	MatrixXd H = Ac.transpose() * Bc;


	JacobiSVD<MatrixXd> svd(H, ComputeThinU | ComputeThinV);

	R = svd.matrixV() * svd.matrixU().transpose();

	if(R.determinant() < 0) {
		MatrixXd X = MatrixXd::Identity(as.size(), bs.size());
		X(as.size() - 1,  bs.size() - 1) = R.determinant();
		R = svd.matrixV() * X * svd.matrixU().transpose();
	}

	t = -R*cA + cB;
}

void correspondence_arrange(const vector<Vector3d> &as, vector<Vector3d> &out, vector<unsigned> &c) {
	out.resize(as.size());
	for(unsigned i = 0; i < c.size(); i++) {
		out[i] = as[c[i]];
	}
}

void correspondence_solve_ideal(const vector<Vector3d> &as, const vector<Vector3d> &bs, vector<unsigned> *c) {

	if(as.size() != bs.size()) {
		printf("Both sets must be complete!\n");
		return;
	}


	/* Compute set centers */
	Vector3d cA = centroid(as), cB = centroid(bs);

	MatrixXd Ac(as.size(), 3), Bc(bs.size(), 3);
	for(unsigned i = 0; i < as.size(); i++){
		Ac.block<1,3>(i, 0) = as[i] - cA;
		Bc.block<1,3>(i, 0) = bs[i] - cB;
	}


	// Gets the eigenvalues/vectors in increasing order
	SelfAdjointEigenSolver<MatrixXd> eigA(Ac * Ac.transpose()),
									 eigB(Bc * Bc.transpose());

	MatrixXd vecsA = eigA.eigenvectors(),
			 vecsB = eigB.eigenvectors();

	MatrixXd Qa(as.size(), 3), Qb(bs.size(), 3);
	for(int i = 0; i < 3; i++) {
		// For now we take the absolute value of each eigenvector to deal with sign inconsistency
		Qa.col(i) = vecsA.col(vecsA.cols() - i - 1).array().abs().matrix();
		Qb.col(i) = vecsB.col(vecsB.cols() - i - 1).array().abs().matrix();
	}


	// MatrixXd P = Qa * Qb.transpose(); // eigA.eigenvectors() * eigB.eigenvectors().transpose();

	// 'Feature vector' are the columns of these
	//cout << Qa.transpose() << endl << endl;
	//cout << Qb.transpose() << endl << endl;

	c->resize(as.size());

	// Determine column permutation matrix based on best matches
	//MatrixXd P = MatrixXd::Zero(as.size(), bs.size());
	for(unsigned i = 0; i < as.size(); i++) {

		double bestE = DBL_MAX;
		int bestJ = 0;

		for(unsigned j = 0; j < bs.size(); j++) {
			double e = (Qa.block<1,3>(i, 0) - Qb.block<1,3>(j, 0)).squaredNorm();

			if(e < bestE) {
				bestE = e;
				bestJ = j;
			}
		}

		(*c)[i] = bestJ;

		//P(bestJ, i) = 1;
	}


	//if(P.rank() != as.size()) {
	//	// the matches were probably not sufficiently distinct
	//	// TODO: Ratio test?
//
//		cout << "failed" << endl;
//	}


	//cout << P << endl;

}


}
