#ifndef TANSA_ALGORITHM_H_
#define TANSA_ALGORITHM_H_

#include "core.h"

#include <vector>


namespace tansa {

/**
 * Computes the center point of a set of points
 */
inline Vector3d centroid(const std::vector<Vector3d> &ps) {
	Vector3d c = Vector3d::Zero();
	for(unsigned i = 0; i < ps.size(); i++) {
		c = c + ps[i];
	}

	return c / ps.size();
}


/**
 * Pretty standard SVD based recovery of labeled point set rigid transformation recovery
 * Computes bs[i] = R as[i] + t
 */
void rigid_transform_solve(const std::vector<Vector3d> &as, const std::vector<Vector3d> &bs, Matrix3d &R, Vector3d &t, const std::vector<double> &w = {});


/**
 * Solves the optimal assignment problem. Internally this uses the Munkres (aka Hungarian) Algorithm
 */
class AssignmentSolver {
public:

	/**
	 * Create a new solver instance
	 *
	 * @param epsilon the largest floating point value that should be considered zero
	 */
	AssignmentSolver(double epsilon = 1e-5) { this->epsilon = epsilon; }

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

	double epsilon;

	unsigned step;
	MatrixXd W; /**< The square working matrix */
	MatrixXi M; /**< Current entry marking */
	std::vector<bool> rowLines;
	std::vector<bool> colLines;

	std::vector<Vector2i> path;


};


/**
 * Solves the correspondence problem for sets of points subject to rigid motion (not scaling or skewing)
 */
class RigidPointCorrespondenceSolver {
public:

	// TODO: We'd like to store some data for it so we can quickly run against a single dataset on multiple different point subsets
	//RigidPointCorrespondenceSolver(const std::vector<Vector3d> &as);

	/**
	 * Given two sets of unlabeled points, this will compute the transform that best matches them,
	 * So: b_j = M * a_i
	 *
	 * Based on approach in 'Determining Correspondences and Rigid Motion of 3-D Point Sets with Missing Data' by Wang et al.
	 * This ideal version assumes that there are no missing or outlier points
	 *
	 * @param as first set of points
	 * @param bs second set of points
	 * @param c the correspondences. such that bs[c[i]] = as[i]. In these version, all the indices should be >= 0
	 * @param ideal whether or not to use ideal mode
	 */
	bool solve(const std::vector<Vector3d> &as, const std::vector<Vector3d> &bs, std::vector<int> *c, bool ideal = false);

	/**
	 * Rearranges the 'bs' set from correspondence_solve_ideal to match the ordering of the points in 'as'
	 */
	void arrange(const std::vector<Vector3d> &bs, const std::vector<int> &c, std::vector<Vector3d> *out);



private:

	void decompose_eigenstructure(const MatrixXd &M, Vector3d *d, MatrixXd *Q);

	// These solve a single iteration of the algorithm given feature matrices and return the cost of the found correspondes
	double solve_ideal(const MatrixXd &Qa, const MatrixXd &Qb, std::vector<int> *c);
	double solve_general(const Vector3d &dA, const MatrixXd &Qa, const Vector3d &dB, const MatrixXd &Qb, std::vector<int> *c);

};


struct IterativeClosestPointOptions {
	double max_error = 0.02; /**< The largest average amount of error that should be allowed between the best alignment and the original query set */

	double max_distance = 0.4; /**< Only match to points within this distance of the original points */

	double max_iterations = 3; /**< Max number of refinement iteratiosn to use. 3 is the recommended minimum for some outlier rejection */
};


class IterativeClosestPoint {
public:
	IterativeClosestPoint(const IterativeClosestPointOptions &options = IterativeClosestPointOptions()) { this->options = options; }

	bool align(const std::vector<Vector3d> &query, const std::vector<Vector3d> &pts, Matrix3d *R, Vector3d *t, std::vector<int> *indices);

private:
	IterativeClosestPointOptions options;
};


}

#endif
