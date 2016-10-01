/*
	Contains the generator of a minimum snap trajectory through points in the differentially flat space [x, y, z, yaw]
	See 'Polynomial Trajectory Planning for Quadrotor Flight' by Richter et. al. for a full derivation.

	Given:
	- A desired polynomial order
	- A set of points [p0, pN]
	- Desired velocities through v0 and vN
*/


/*
	Generates a piecewise polynomial trajectory with minimal snap through the given waypoints at the given times

	Currently this assumes we start at rest and end at rest
*/
void compute_min_snap(const vector<Vector4d> &x, const vector<double> &t) {

	int N = 9; // Polynomial order for generated segments : c_n * t^n + c_n-1 * t^(n-1) + ... + c_0

	int M = x.size(); // number of segments


	/*
		Our QP is of the form:
		argmin p^T Q p
		s.t. A p - b = 0
	*/

	// The whole cost matrix for a single axis
	// Note: this stays the same for each axis as this is only dependent on n, m, and t
	MatrixXd Q = MatrixXd::Zero(N * M, N * M);

	for(int s = 0; s < M; s++) { // s is the index of the current segment
		// Sub-cost matrix for derivatives of this segment
		MatrixXd Qs(N, N);

		// The order of the derivative we are minimizing
		// Currently the cost of all other derivates is 0
		int r = 4;

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
	int nc = ; // Number of contraints


	MatrixXd A(nc, N * M);
	VectorXd b(N * M);

}



// Exmple code for solving the QP:


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
typedef CGAL::Quadratic_program<int> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

int main() {
	// by default, we have a nonnegative QP with Ax <= b
	Program qp(CGAL::SMALLER, true, 0, false, 0);

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
