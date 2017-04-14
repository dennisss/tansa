#ifndef TANSA_ALGORITHM_H_
#define TANSA_ALGORITHM_H_

#include "core.h"

#include <vector>


namespace tansa {


/**
 * Solves the optimal assignment problem. Internally this uses the Munkres (aka Hungarian) Algorithm
 */
class AssignmentSolver {
public:

	/**
	 * Does the solving
	 *
	 * @param w the weights associated with assignment. This can be of any size N x M
	 * @param c the output assignments of each row index. -1 if the row could not be assigned
	 * @return the total cost of the found assignment
	 */
	double solve(const MatrixXd &w, std::vector<int> *c);


private:


	void solve_step1();
	void solve_step2();
	void solve_step3();
	void solve_step4();
	void solve_step5();
	void solve_step6();

	bool find_uncovered_zero(unsigned *i, unsigned *j);
	bool find_in_row(int type, unsigned i, unsigned *j);
	bool find_in_col(int type, unsigned *i, unsigned j);
	void augment_path();
	void reset_cover();
	void reset_primes();


	unsigned step;
	MatrixXd W; /**< The square working matrix */
	MatrixXi M; /**< Current entry marking */
	std::vector<bool> rowLines;
	std::vector<bool> colLines;

	std::vector<Vector2i> path;


};


}

#endif
