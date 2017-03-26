#ifndef TANSA_TRAJECTORY_UTILS_H_
#define TANSA_TRAJECTORY_UTILS_H_


/**
 * Computes a derivative based on the 0th order evaluation
 * So given [t^0, t^1, t^2, t^3, t^4 ...]
 * Computes [0, 1*t^0, 2*t^1, 3*t^2, ...   ] for n = 1
 */
inline VectorXd diffvec(const VectorXd t, int n) {

	VectorXd dt = VectorXd::Zero(t.size());

	for(int i = n; i < t.size(); i++) {
		double c = 1;
		for(int p = i; p > i - n; p--) {
			c *= p;
		}

		dt(i) = c * t(i - n);
	}

	return dt;
}

/**
 * Given t, computes [t^0, t^1, t^2, ...t^(n-1)]
 */
inline VectorXd powvec(double t, int n) {
	VectorXd v(n);
	v(0) = 1;
	for(int i = 1; i < n; i++) {
		v(i) = v(i - 1) * t;
	}

	return v;
}


#endif
