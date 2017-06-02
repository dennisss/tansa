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




class DisjointSets {
public:

	/**
	 * Creates a new collection of disconnected elements
	 */
	DisjointSets(unsigned N) {
		elements.resize(N);
		this->clear();
	}

	inline void clear() {
		for(unsigned i = 0; i < elements.size(); i++) {
			makeSet(i);
		}
	}

	inline void makeSet(unsigned x) {
		ElementData &e_x = elements[x];
		e_x.rank = 0;
		e_x.parent = x;
		e_x.min = x;
	}

	inline unsigned findSet(unsigned x) {
		ElementData &e_x = elements[x];

		if(e_x.parent != x)
			e_x.parent = findSet(e_x.parent);
		return e_x.parent;
	}

	inline unsigned findSetMin(unsigned x) {
		ElementData &e_x = elements[x];

		x = findSet(x);
		return e_x.min;
	}

	inline void unionSets(unsigned x, unsigned y) {
		// Find root labels
		x = findSet(x);
		y = findSet(y);

		// if x and y are already in the same set (i.e., have the same root or representative)
		if(x == y)
			return;

		ElementData &e_x = elements[x],
					&e_y = elements[y];

		// x and y are not in same set, so we merge them
		if(e_x.rank < e_y.rank)
			e_x.parent = y;
		else if(e_x.rank > e_y.rank)
			e_y.parent = x;
		else {
			e_y.parent = x;
			e_x.rank = e_x.rank + 1;
		}


		// Update the min in the root of the new set
		if(e_x.min < e_y.min) {
			e_y.min = e_x.min;
		}
		else {
			e_x.min = e_y.min;
		}
	}

private:

	struct ElementData {
		unsigned parent;
		unsigned min;
		unsigned rank;
	};

	std::vector<ElementData> elements;

};



}

#endif
