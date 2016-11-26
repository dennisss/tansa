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
typedef CGAL::Quadratic_program<int> Program; // TODO: This will need to change
typedef CGAL::Quadratic_program_solution<ET> Solution;


/*
	Contains the generator of a minimum snap trajectory through points in the differentially flat space [x, y, z, yaw]

	-

	Given:
	- A desired polynomial order
	- A set of points [p0, pN]
	- Desired velocities through v0 and vN
*/


/**
 * Generates a piecewise polynomial trajectory with minimal snap through the given waypoints at the given times
 *
 * See 'Polynomial Trajectory Planning for Quadrotor Flight' by Richter et. al. for a full derivation.
 * This is the constrained QP formulation
 *
 * Currently this assumes we start at rest and end at rest
 */
void compute_min_snap(const vector<Point> &x, const vector<double> &t) {

	// Number of coefficients generated segments : c_(n-1) * t^(n-1) + c_(n-2) * t^(n-2) + ... + c_0
	// Note that in the Richter paper, big N is the polynomial order (N-1)
	int N = 10;

	int M = x.size() - 1; // number of segments

	if(t.size() != x.size()) {
		printf("Wrong number of times provided\n");
		return;
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

		// The order of the derivative we are minimizing
		// Currently the cost of all other derivates is 0
		int r = R;

		for(int i = 0; i < N; i++) { // i is the current row
			for(int l = 0; l < N; l++) {
				if(i < r || l < r) {
					Qs(i, l) = 0;
					continue;
				}
				// else i >= r && l >= r

				int pi = 1;
				for(int m = 0; m < r - 1; m++) {
					pi *= (i - m) * (l - m);
				}

				// Duration of this segment
				double T = t[i+1] - t[i];

				int e = i + l - 2*r + 1;
				Qs(i, l) = 2.0 * ((double) pi) * pow(T, e) / ((double) e);
			}
		}

		// Insert into full matrix
		Q.block(N*s, N*s, N, N) = Qs;
	}


	// Build constraints (one axis at a time)
	// Of the form Ax = b
	// - start and stop: pos, vel, accel, jerk constraints
	// - continuitity of all intermediate positions derivatives
	// Note: A fixed position for an internal point would appear twice
	// - so we will have two of most of the position
	// -> (M+1)*(R-1) + 
	// -> (M+1)*r
	int nc = (M+1)*R; // Number of contraints


	// Iterate over each axis
	for(int d = 0; d < PointDims; d++) {

		// Setup constraints

		MatrixXd A = MatrixXd::Zero(nc, N * M);
		VectorXd b = MatrixXd::Zero(nc);


		for(int i = 0; i < M; i++) {

			// Start of end time vectors for this segment
			VectorXd tvecS = powvec(0, N);
			VectorXd tvecE = powvec(t[i+1] - t[i]);

			// First R rows for the start
			for(int j = 0; j < R; j++) {
				//
				A.block<> = diffvec(tvecS, j)

			}


		}


	}


	// essentially, I need to reuse the diffvec() stuff from polynomial.cpp to set the d

	// continuity constraints should look something like [0, 0, .., Aderiv, -Bderiv, .. 0, 0,] = [0]




}

/*
	Our QP is of the form:
	argmin p^T Q p
	s.t. A p - b = 0

	In CGAL
	Q becomes D
*/
VectorXd qp_solve(MatrixXd Q, MatrixXd A, VectorXd b) {

	// Solving the quadratic program

	// by default, we have a nonnegative QP with Ax <= b
	Program qp(CGAL::EQUAL, true, 0, false, 0);

	// now set the non-default entries:
	const int X = 0;
	const int Y = 1;
	qp.set_a(X, 0,  1); qp.set_a(Y, 0, 1); qp.set_b(0, 7);  //  x + y  <= 7
	qp.set_a(X, 1, -1); qp.set_a(Y, 1, 2); qp.set_b(1, 4);  // -x + 2y <= 4
	qp.set_u(Y, true, 4);                                   //       y <= 4
	qp.set_d(X, X, 2); qp.set_d (Y, Y, 8); // !!specify 2D!!    x^2 + 4 y^2
	qp.set_c(Y, -32);                                       // -32y
	qp.set_c0(64);                                          // +64
	// solve the program, using ET as the exact type
	Solution s = CGAL::solve_quadratic_program(qp, ET());
	assert (s.solves_quadratic_program(qp));
	// output solution
	std::cout << s;
	return 0;

}
