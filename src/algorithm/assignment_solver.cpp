#include <tansa/algorithm.h>

#include <cfloat>

#include <iostream>


using namespace std;

/*
	See http://csclab.murraystate.edu/~bob.pilgrim/445/munkres.html for a great reference
*/

namespace tansa {


#define IS_ZERO(i, j) (fabs(W(i, j)) < epsilon)
#define COVERED(i, j) (rowLines[i] || colLines[j])
#define MAX(a, b) ((a) > (b)? (a) : (b))

#define STAR 1
#define PRIME 2


double AssignmentSolver::solve(const MatrixXd &w, std::vector<int> *c) {

	// Padding with zeros to be a square matrix with the same optimal assignments
	unsigned N = MAX(w.rows(), w.cols());
	W = MatrixXd::Zero(N, N);
	W.block(0, 0, w.rows(), w.cols()) = w;

	M = MatrixXi::Zero(W.rows(), W.cols());

	rowLines = vector<bool>(W.rows(), false);
	colLines = vector<bool>(W.cols(), false);


	// Do the solving
	step = 1;
	unsigned it = 0;
	while(step < 7 && it < W.rows() * 7) {
		switch(step) {
			case 1: solve_step1(); break;
			case 2: solve_step2(); break;
			case 3: solve_step3(); break;
			case 4: solve_step4(); break;
			case 5: solve_step5(); break;
			case 6: solve_step6(); break;
		}

		it++;
	}


	// Extract final assignment
	double cost = 0;
	c->resize(w.rows());
	for(unsigned i = 0; i < w.rows(); i++) {

		unsigned j;
		find_in_row(STAR, i, &j);

		if(j < w.cols())
			(*c)[i] = j;
		else
			(*c)[i] = -1;

		cost += w(i, j);
	}

	return cost;
}


void AssignmentSolver::solve_step1() { // Subtract row mins
	unsigned i, j;
	for(i = 0; i < W.rows(); i++) {
		// Find min
		double min = DBL_MAX;
		for(j = 0; j < W.cols(); j++) {
			if(W(i, j) < min)
				min = W(i, j);
		}

		// Subtract min
		for(j = 0; j < W.cols(); j++)
			W(i, j) -= min;
	}

	step = 2;
}

void AssignmentSolver::solve_step2() { // Star all zeros not already in a row/column with a stared value
	unsigned i, j;

	for(i = 0; i < W.rows(); i++) {
		for(j = 0; j < W.cols(); j++) {
			if(IS_ZERO(i, j) && !COVERED(i, j)) {
				M(i, j) = STAR;
				rowLines[i] = true;
				colLines[j] = true;
			}
		}
	}

	reset_cover();
	step = 3;
}

void AssignmentSolver::solve_step3() { // Check if we've covered everything
	unsigned i, j;

	for(i = 0; i < W.rows(); i++) {
		for(j = 0; j < W.cols(); j++) {
			if(M(i, j) == STAR) {
				colLines[j] = true;
			}
		}
	}

	unsigned n = 0;
	for(i = 0; i < colLines.size(); i++) {
		if(colLines[i])
			n++;
	}

	if(n == M.cols()) {
		step = 7;
	}
	else {
		step = 4;
	}
}

void AssignmentSolver::solve_step4() {

	unsigned i, j;

	while(true) {

		if(!find_uncovered_zero(&i, &j)) {
			step = 6;
			return;
		}
		else {
			M(i, j) = PRIME;

			if(find_in_row(STAR, i, &j)) {
				rowLines[i] = true;
				colLines[j] = false;
			}
			else {
				// Save for the next step
				path.resize(0);
				path.push_back({ i, j });

				step = 5;
				return;
			}
		}
	}
}




void AssignmentSolver::solve_step5() {

	unsigned i, j;

	// Expanding the path created in the last step

	while(true) {

		j = path[path.size() - 1](1);

		if(find_in_col(STAR, &i, j)) {
			path.push_back({ i, j });
		}
		else
			break;

		find_in_row(PRIME, i, &j);
		path.push_back({i, j});
	}


	augment_path();
	reset_cover();
	reset_primes();
	step = 3;
}

void AssignmentSolver::solve_step6() {
	unsigned i, j;

	// Find smallest uncovered value
	double min = DBL_MAX;
	for(i = 0; i < W.rows(); i++) {
		for(j = 0; j < W.cols(); j++) {
			if(!COVERED(i, j) && W(i, j) < min) {
				min = W(i, j);
			}
		}
	}

	for(i = 0; i < W.rows(); i++) {
		for(j = 0; j < W.cols(); j++) {
			if(rowLines[i])
				W(i, j) += min;
			if(!colLines[j])
				W(i, j) -= min;
		}
	}

	step = 4;
}

bool AssignmentSolver::find_uncovered_zero(unsigned *i, unsigned *j) {
	for(unsigned ii = 0; ii < W.rows(); ii++) {
		for(unsigned jj = 0; jj < W.cols(); jj++) {
			if(IS_ZERO(ii, jj) && !COVERED(ii, jj)) {
				*i = ii;
				*j = jj;
				return true;
			}
		}
	}

	return false;
}

bool AssignmentSolver::find_in_row(int type, unsigned i, unsigned *j) {
	for(unsigned jj = 0; jj < M.cols(); jj++) {
		if(M(i, jj) == type) {
			*j = jj;
			return true;
		}
	}

	return false;
}

bool AssignmentSolver::find_in_col(int type, unsigned *i, unsigned j) {
	for(unsigned ii = 0; ii < M.rows(); ii++) {
		if(M(ii, j) == type) {
			*i = ii;
			return true;
		}
	}

	return false;
}

void AssignmentSolver::augment_path() {
	unsigned i, j;
	for(unsigned p = 0; p < path.size(); p++) {
		i = path[p](0), j = path[p](1);

		if(M(i, j) == STAR)
			M(i, j) = 0;
		else
			M(i, j) = STAR;
	}
}

void AssignmentSolver::reset_cover() {
	// Reset lines
	for(unsigned i = 0; i < W.rows(); i++) {
		rowLines[i] = false;
		colLines[i] = false;
	}
}

void AssignmentSolver::reset_primes() {
	for(unsigned i = 0; i < M.rows(); i++) {
		for(unsigned j = 0; j < M.cols(); j++) {
			if(M(i, j) == PRIME) {
				M(i, j) = 0;
			}
		}
	}
}



}
