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


bool qp_solve_cplex(const MatrixXd &Q, const MatrixXd &A, const VectorXd &b, const vector<CGAL::Comparison_result> &rels, VectorXd &x, double &cost);


namespace tansa {


bool qp_solve(const MatrixXd &Q, const MatrixXd &A, const VectorXd &b, const vector<CGAL::Comparison_result> &rels, VectorXd &x, double &cost);


bool compute_minsnap_mellinger11(const vector<ConstrainedPoint> &x, const vector<double> &t, const vector<double> &corridors, Trajectory::Ptr *out, double *cost) {

	// Number of coefficients generated segments : c_(n-1) * t^(n-1) + c_(n-2) * t^(n-2) + ... + c_0
	// Note that in the Richter paper, big N is the polynomial order (N-1)
	int N = 10;

	// Number of intermediate points for each corridor
	int n_intermediate = 8;

	int M = x.size() - 1; // number of segments

	if(t.size() != x.size()) {
		printf("Wrong number of times provided\n");
		return false;
	}

	if(corridors.size() > M) {
		printf("Too many corridors\n");
		return false;
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
	VectorXd costs(PointDims);

	// Iterate over each axis
	for(unsigned d = 0; d < PointDims; d++) {

		// For holding the constrains (it is easier to vectorize them them put them in a matrix right away)
		vector<MatrixXd> Arows;
		vector<double> brows;
		vector<CGAL::Comparison_result> rels; // relation between A and b

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
					rels.push_back(CGAL::EQUAL);
				}
				if(endConstrained) {
					MatrixXd a = EmptyConstraintRow;
					a.block(0, i*N, 1, N) = diffvec(tvecE, j).transpose();
					double b = x[i+1][j](d);
					Arows.push_back(a); brows.push_back(b);
					rels.push_back(CGAL::EQUAL);
				}

				// Add continuity constraint for all internal points between segments
				if(!endConstrained && i < M - 1) {
					MatrixXd a = EmptyConstraintRow;
					a.block(0, i*N, 1, N) = diffvec(tvecE, j).transpose(); // +derivative at end
					a.block(0, (i+1)*N, 1, N) = -1.0 * diffvec(tvecS, j).transpose(); // this technically is wrong but works as tvecS is constant between segments (but really should be recomputed for the next segment)

					double b = 0;
					Arows.push_back(a); brows.push_back(b);
					rels.push_back(CGAL::EQUAL);

				}
			}


			// Adding corridor constraint
			if(i < corridors.size() && corridors[i] > 0) {

				// Unit vector along segment
				Vector3d ti = (x[i+1][0] - x[i][0]).normalized();

				// Loop through intermediate points;
				for(int j = 0; j < n_intermediate; j++) {

					// Time for this intermediate point
					double tj = ((1.0*j) / (1 + n_intermediate)) * (t[i+1] - t[i]);

					// Vectorized powers of time (represents the matrix coefficients for position)
					VectorXd tvec = powvec(tj, N);


					// distance from segment:
					// -> (p(t) - waypoint) - ((p(t) - waypoint) dot ray)
					// -> (tvec - waypoint_x) - (tvec * ray_x) - waypoint * ray_x <= delta
					// (tvec - tvec*ray_x) <= delta + waypoint_x + (waypoint_x * ray_x)
					MatrixXd a = EmptyConstraintRow;
					a.block(0, i*N, 1, N) = diffvec(tvec, 0); // * (1 - x[i][0](d));
					double bp = corridors[i] + x[i][0](d) * (1 + ti(d)); // for d <= delta
					double bn = -corridors[i] + x[i][0](d) * (1 + ti(d)); // for d >= -delta

					Arows.push_back(a); brows.push_back(bp); rels.push_back(CGAL::SMALLER);
					Arows.push_back(a); brows.push_back(bn); rels.push_back(CGAL::LARGER);
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

		VectorXd x;
		double cost;

#ifdef USE_CPLEX
		bool success = qp_solve_cplex(Q, A, b, rels, x, cost);
#else
		bool success = qp_solve(Q, A, b, rels, x, cost);
#endif

		if(!success) {
			return false;
		}

		xs.push_back(x);
		costs(d) = cost;
	}


	// Creating the trajectory objects
	vector<Trajectory::Ptr> segs;
	for(int i = 0; i < M; i++) {
		VectorXd cs[PointDims];
		for(int d = 0; d < PointDims; d++) {
			cs[d] = xs[d].segment(i*N, N);
		}

		Trajectory::Ptr poly( new PolynomialTrajectory(cs, 0, t[i+1] - t[i]) );
		segs.push_back( poly );
	}

	if(out != NULL)
		*out = make_shared<PiecewiseTrajectory>( segs, t[0], t[t.size() - 1] );

	if(cost != NULL)
		*cost = costs.norm();

	return true;
}

bool compute_minsnap_optimal_mellinger11(const vector<ConstrainedPoint> &x, double ts, double te, vector<double> corridors, Trajectory::Ptr *out) {

	// Number of segments
	int m = x.size() - 1;

	// Current best trajectory
	vector<double> t;
	Trajectory::Ptr tr;
	double f;

	// Initially try a uniform time distribution
	for(int i = 0; i < x.size(); i++) {
		t.push_back(ts + i * ((te - ts) / m) ); // TODO: Instead base this on the distance between points
	}

	// Compute initial cost
	if(!compute_minsnap_mellinger11(x, t, corridors, &tr, &f))
		return false;

	// Set of all directions in which we will descend
	vector<VectorXd> g;
	for(int i = 0; i < m; i++) {
		VectorXd gi(m);

		for(int j = 0; j < m; j++) {
			gi(j) = i == j? 1.0 : (- 1.0 / (m - 1));
		}

		g.push_back(gi);
	}

	// Max number of iterations to do
	int maxit = 7;
	// Current incrementation step in seconds : initially set to half the average segment length
	double h = 0.5 * ((te - ts) / m);

	for(int it = 0; it < maxit; it++) {

		// starting time vector for this iteration
		vector<double> T = t;

		// Try every direction
		for(int i = 0; i < g.size(); i++) {
			Trajectory::Ptr trI;
			double fI;

			vector<double> ti;
			ti.push_back(ts);
			bool valid = true;
			for(int j = 1; j < x.size(); j++) {
				// Duration of this segment
				double tseg = (T[j] - T[j - 1]) + h*g[i](j - 1);

				// If negative, then it is impossible. Also if really small then we are probably going too fast and numerically unstable
				if(tseg <= 0.01) {
					valid = false;
					break;
				}

				ti.push_back(ti[j - 1] + tseg);
			}

			cout << valid << endl;

			if(!valid || !compute_minsnap_mellinger11(x, ti, corridors, &trI, &fI)) {
				continue;
			}

			cout << fI << endl;

			if(fI < f) {
				tr = trI;
				t = ti;
				f = fI;
			}
		}

		/*
		// Cost changed by less than 1%: stop early
		if(abs(f - F) / F < 0.01) {
			break;
		}
		*/

		// Decrease step size
		h *= 0.6;
	}

	/*
	for(auto ti : t ) {
		cout << ti << " ";
	}
	cout << endl;
	*/

	*out = tr;

	return true;
}


/*
	Our QP is of the form:
	argmin p^T Q p
	s.t. A p - b = 0

	In CGAL
	Q becomes D
*/

bool qp_solve(const MatrixXd &Q, const MatrixXd &A, const VectorXd &b, const vector<CGAL::Comparison_result> &rels, VectorXd &x, double &cost) {

	Program qp(CGAL::EQUAL, false, 0, false, 0);


	// Copying values into program
	for(int i = 0; i < Q.rows(); i++) {
		for(int j = 0; j <= i; j++) {
			qp.set_d(i, j, Q(i, j));
		}
	}

	for(int i = 0; i < A.rows(); i++) {
		for(int j = 0; j < A.cols(); j++) {
			qp.set_a(j, i, A(i, j));
		}
		qp.set_b(i, b(i));
		qp.set_r(i, rels[i]);
	}

	// solve the program, using ET as the exact type
	Solution s = CGAL::solve_quadratic_program(qp, ET());

	if(s.is_infeasible() || !s.solves_quadratic_program(qp)) {
		return false;
	}

	// output solution
	//std::cout << s << endl;

	x.resize(A.cols());
	int i = 0;
	for(auto it = s.variable_values_begin(); it != s.variable_values_end(); it++) {
		x(i) = CGAL::to_double(*it);
		i++;
	}

	cost = CGAL::to_double(s.objective_value());

	return true;

}


}

#ifdef USE_CPLEX

#define IL_STD

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

bool qp_solve_cplex(const MatrixXd &Q, const MatrixXd &A, const VectorXd &b, const vector<CGAL::Comparison_result> &rels, VectorXd &x, double &cost) {

	IloEnv env;

	IloModel model(env);
	IloNumVarArray vars(env);
	IloRangeArray cons(env);

	// Adding all variables
	for(int i = 0; i < Q.rows(); i++) {
		vars.add(IloNumVar(env, -IloInfinity, IloInfinity));
	}

	// Adding objective matrix
	IloExpr expr(env);
	for(int i = 0; i < Q.rows(); i++) {
		for(int j = 0; j < Q.cols(); j++) {
			expr += Q(i, j) * vars[i]*vars[j];
		}
	}
	model.add(IloMinimize(env, expr));

	//cout << "Vars: " << Q.rows() << endl;
	//cout << "Constraints: " << A.rows() << endl;

	// Adding linear constraints
	for(int i = 0; i < A.rows(); i++) {
		IloExpr e(env);
		for(int j = 0; j < A.cols(); j++) {
			e += A(i, j) * vars[j];
		}

		if(rels[i] == CGAL::EQUAL)
			cons.add(IloRange(env, b(i), e, b(i)));
		else if(rels[i] == CGAL::LARGER)
			cons.add(e >= b(i));
		else if(rels[i] == CGAL::SMALLER) {
			cons.add(e <= b(i));
		}
	}
	model.add(cons);




	IloCplex cplex(model);
	cplex.setParam(IloCplex::RootAlg, IloCplex::Network); // ::Primal
	cplex.setOut(env.getNullStream());

	//cplex.exportModel("axis.lp");

	// Optimize the problem and obtain solution.
	if(!cplex.solve()) {
		env.error() << "Failed to optimize LP" << endl;
		return false;
	}

	IloNumArray vals(env);
	cplex.getValues(vals, vars);
	x.resize(A.cols());
	for(int i = 0; i < A.cols(); i++) {
		x(i) = vals[i];
	}

	cost = cplex.getObjValue();

	env.end();

	return true;
}

#endif
