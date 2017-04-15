#include <tansa/algorithm.h>

#include <vector>
#include <iostream>
#include <cfloat>

#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

namespace tansa {

void RigidPointCorrespondenceSolver::arrange(const vector<Vector3d> &bs, const vector<int> &c, vector<Vector3d> *out) {
	out->resize(c.size());
	for(unsigned i = 0; i < c.size(); i++) {
		(*out)[i] = bs[c[i]];
	}
}


bool RigidPointCorrespondenceSolver::solve(const vector<Vector3d> &as, const vector<Vector3d> &bs, vector<int> *c, bool ideal) {

	if(as.size() != bs.size() && ideal) {
		printf("Both sets must be complete!\n");
		return false;
	}


	/* Compute set centers */
	Vector3d cA = centroid(as), cB = centroid(bs);

	MatrixXd Ac(as.size(), 3), Bc(bs.size(), 3);
	for(unsigned i = 0; i < as.size(); i++){
		Ac.block<1,3>(i, 0) = as[i] - cA; // TODO: Don't these need to be tranposed
		Bc.block<1,3>(i, 0) = bs[i] - cB;
	}


	// Get the eigenvalues and eigenvectors of the scatter matrices
	Vector3d dA, dB;
	MatrixXd Qa(as.size(), 3), Qb(bs.size(), 3);
	decompose_eigenstructure(Ac * Ac.transpose(), &dA, &Qa);
	decompose_eigenstructure(Bc * Bc.transpose(), &dB, &Qb);


	// There will potentially be sign inconsistency in the eigenvectors
	// Therefore we must consider all possible permutations on the signs of one of the eigenvectors
	// We put the permutation that results in the lowest error
	// TODO: If we wanted a quick and 'ok' solution, we could take the absolute value of both eigenvector sets and simplify this to only one sign iteration
	// TODO: Ratio test best and second best matches
	double minCost = DBL_MAX;
	vector<int> minCorresp;
	vector<int> corresp(as.size(), -1);
	for(unsigned it = 0; it < 8; it++) {
		RowVector3d signA(it & 0b001? 1 : -1, it & 0b010? 1 : -1, it & 0b100? 1 : -1);
		cout << signA << endl;

		// Make a copy of Qa with the currently proposed signs
		MatrixXd Qai(Qa.rows(), Qa.cols());
		for(unsigned i = 0; i < Qa.rows(); i++) {
			Qai.row(i) = Qa.row(i).cwiseProduct(signA);
		}

		double cost = ideal? solve_ideal(Qai, Qb, &corresp) : solve_general(dA, Qai, dB, Qb, &corresp);
		if(cost < minCost) {
			minCorresp = corresp;
			minCost = cost;
		}
	}


	*c = minCorresp;
	return true;
}

void RigidPointCorrespondenceSolver::decompose_eigenstructure(const MatrixXd &M, Vector3d *d, MatrixXd *Q) {
	// Gets the eigenvalues/vectors in increasing order
	SelfAdjointEigenSolver<MatrixXd> eig(M);

	MatrixXd vecs = eig.eigenvectors();
	VectorXd vals = eig.eigenvalues();

	// We assume that we are given one of the scatter matrices which will definitely be positive semi-definite (so no negative eigenvalues)
	// Take last three values/vectors
	for(unsigned i = 0; i < 3; i++) {
		unsigned idx = vecs.cols() - i - 1;
		Q->col(i) = vecs.col(idx);
		(*d)(i) = vals(idx);
	}
}

double RigidPointCorrespondenceSolver::solve_ideal(const MatrixXd &Qa, const MatrixXd &Qb, std::vector<int> *c) {

// TODO:
	// MatrixXd P = Qa * Qb.transpose(); // eigA.eigenvectors() * eigB.eigenvectors().transpose();

	// 'Feature vector' are the columns of these
	//cout << Qa.transpose() << endl << endl;
	//cout << Qb.transpose() << endl << endl;

	// Determine column permutation matrix based on best matches
	//MatrixXd P = MatrixXd::Zero(as.size(), bs.size());
	// TODO: What if two i indices have the same best
	double cost = 0;
	for(unsigned i = 0; i < Qa.rows(); i++) {

		double bestE = DBL_MAX;
		int bestJ = 0;

		for(unsigned j = 0; j < Qb.rows(); j++) {
			double e = (Qa.block<1,3>(i, 0) - Qb.block<1,3>(j, 0)).squaredNorm();

			if(e < bestE) {
				bestE = e;
				bestJ = j;
			}
		}

		(*c)[i] = bestJ;
		cost += bestE;

		//P(bestJ, i) = 1;
	}

	//if(P.rank() != as.size()) {
	//	// the matches were probably not sufficiently distinct
	//	// TODO: Ratio test?
	//
	//		cout << "failed" << endl;
	//	}


	return cost;
}


double RigidPointCorrespondenceSolver::solve_general(const Vector3d &dA, const MatrixXd &Qa, const Vector3d &dB, const MatrixXd &Qb, std::vector<int> *c) {

	MatrixXd h(Qa.rows(), Qb.rows()); // Affinity matrix

	for(int i = 0; i < h.rows(); i++) {
		for(int j = 0; j < h.cols(); j++) {

			double s = 0;
			for(int k = 0; k < 3; k++) {
				// Eigenvalues corresponding to the vectors
				double dAk = dA(k),
					   dBk = dB(k);

				double pik = Qa(i, k),
					   qjk = Qb(j, k);

				s += dAk * dBk * pow(abs(pik - qjk), 2);
			}

			// We do not need the negative sign as in the paper if the assignment algorithm is minimizing
			h(i, j) = s; // -s;
		}
	}

	AssignmentSolver as;
	return as.solve(h, c);


	// TODO: Also implement the outlier rejection method
}



}
