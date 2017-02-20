#include <tansa/trajectory.h>
#include "utils.h"

#include <iostream>
#include <cassert>

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif
// program and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;



namespace tansa {


VectorXd qp_solve(MatrixXd Q, MatrixXd A, VectorXd b);


Trajectory *compute_minsnap_mellinger11(const vector<ConstrainedPoint> &x, const vector<double> &t, vector<double> corridors) {

	// Number of coefficients generated segments : c_(n-1) * t^(n-1) + c_(n-2) * t^(n-2) + ... + c_0
	// Note that in the Richter paper, big N is the polynomial order (N-1)
	int N = 10;

	int M = x.size() - 1; // number of segments

	if(t.size() != x.size()) {
		printf("Wrong number of times provided\n");
		return NULL;
	}

	if(corridors.size() > M) {
		printf("Too many corridors\n");
		return NULL;
	}

	/*
		Our QP is of the form:
		argmin p^T Q p
		s.t. A p - b = 0
	*/

	int R = 4; // Derivative to minimize : in this case snap (0 indexed)

	// The whole cost matrix for a single axis
	// Note: this stays the same for each axis as this is only dependent on n, m, and t
	MatrixXd Q = MatrixXd::Zero(N * M, N * M);

	for(int s = 0; s < M; s++) { // s is the index of the current segment
		// Sub-cost matrix for derivatives of this segment
		MatrixXd Qs(N, N);

		// Duration of this segment
		double T = t[s+1] - t[s];

		// The order of the derivative we are minimizing
		// Currently the cost of all other derivates is 0
		int r = R;

		for(int i = 0; i < N; i++) { // i is the current row
			for(int l = 0; l < N; l++) {
				if(i < r || l < r) {
					Qs(i, l) = 0;
					continue;
				}
				// below: else i >= r && l >= r

				int pi = 1;
				for(int m = 0; m <= r - 1; m++) {
					pi *= (i - m) * (l - m);
				}


				int e = i + l - 2*r + 1;
				Qs(i, l) = 2.0 * ((double) pi) * pow(T, e) / ((double) e);
			}
		}

		// Insert into full matrix
		Q.block(N*s, N*s, N, N) = Qs;
	}




	// Building constraints and solving each axis

	vector<VectorXd> xs;

	// Iterate over each axis
	for(int d = 0; d < PointDims; d++) {

		// For holding the constrains (it is easier to vectorize them them put them in a matrix right away)
		vector<MatrixXd> Arows;
		vector<double> brows;

		#define EmptyConstraintRow MatrixXd::Zero(1, M*N)

		// Fill A
		// Loop over segments
		for(int i = 0; i < M; i++) {

			// Start of end time vectors for this segment
			VectorXd tvecS = powvec(0, N); // This is the same for every segment
			VectorXd tvecE = powvec(t[i+1] - t[i], N);


			// Loop over derivative degree
			for(int j = 0; j <= R; j++) {

				bool startConstrained = x[i].isConstrained(j), // j == 0 || i == 0, //x[i].size() < j,
					 endConstrained = x[i+1].isConstrained(j); // j == 0 || i == M - 1; //x[i+1].size() < j;

				if(startConstrained) {
					MatrixXd a = EmptyConstraintRow;
					a.block(0, i*N, 1, N) = diffvec(tvecS, j).transpose();
					double b = x[i][j](d);
					Arows.push_back(a); brows.push_back(b);
				}
				if(endConstrained) {
					MatrixXd a = EmptyConstraintRow;
					a.block(0, i*N, 1, N) = diffvec(tvecE, j).transpose();
					double b = x[i+1][j](d);
					Arows.push_back(a); brows.push_back(b);
				}

				// Add continuity constraint for all internal points between segments
				if(!endConstrained && i < M - 1) {
					MatrixXd a = EmptyConstraintRow;
					a.block(0, i*N, 1, N) = diffvec(tvecE, j).transpose(); // +derivative at end
					a.block(0, (i+1)*N, 1, N) = -diffvec(tvecS, j).transpose(); // this technically is wrong but works as tvecS is constant between segments (but really should be recomputed for the next segment)

					double b = 0;
					Arows.push_back(a); brows.push_back(b);
				}
			}
		}


		// Setup constraints
		unsigned nc = Arows.size();
		MatrixXd A = MatrixXd::Zero(nc, N * M);
		VectorXd b = VectorXd::Zero(nc);

		for(int i = 0; i < nc; i++) {
			A.block(i, 0, 1, N*M) = Arows[i];
			b(i) = brows[i];
		}


		VectorXd x = qp_solve(Q, A, b);
		xs.push_back(x);
	}


	// Creating the trajectory objects
	vector<Trajectory *> segs;
	for(int i = 0; i < M; i++) {
		VectorXd cs[PointDims];
		for(int d = 0; d < PointDims; d++) {
			cs[d] = xs[d].segment(i*N, N);
		}

		Trajectory *poly = new PolynomialTrajectory(cs, 0, t[i+1] - t[i]);
		segs.push_back( poly );
	}

	return new PiecewiseTrajectory( segs, t[0], t[t.size() - 1] );
}

/*
	Our QP is of the form:
	argmin p^T Q p
	s.t. A p - b = 0

	In CGAL
	Q becomes D
*/

VectorXd qp_solve(MatrixXd Q, MatrixXd A, VectorXd b) {

	Program qp(CGAL::EQUAL, false, 0, false, 0);


	// Copying values into program
	for(int i = 0; i < Q.rows(); i++) {
		for(int j = 0; j < Q.cols(); j++) {
			qp.set_d(i, j, Q(i, j));
		}
	}

	for(int i = 0; i < A.rows(); i++) {
		for(int j = 0; j < A.cols(); j++) {
			qp.set_a(j, i, A(i, j));
		}
		qp.set_b(i, b(i));
	}

	// solve the program, using ET as the exact type
	Solution s = CGAL::solve_quadratic_program(qp, ET());
	assert(s.solves_quadratic_program(qp));
	// output solution
	//std::cout << s << endl;;

	VectorXd x(A.cols());
	int i = 0;
	for(auto it = s.variable_values_begin(); it != s.variable_values_end(); it++) {
		x(i) = CGAL::to_double(*it);
		i++;
	}


	return x;

}


}
