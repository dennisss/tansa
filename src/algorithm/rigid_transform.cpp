#include <tansa/algorithm.h>

using namespace std;


namespace tansa {

void rigid_transform_solve(const vector<Vector3d> &as, const vector<Vector3d> &bs, Matrix3d &R, Vector3d &t, const vector<double> &w) {

	/* Compute set centers */
	Vector3d cA = centroid(as), cB = centroid(bs);

	MatrixXd Ac(as.size(), 3), Bc(bs.size(), 3);
	for(unsigned i = 0; i < as.size(); i++) {
		double wi = 1;
		if(i < w.size()) {
			wi = w[i];
		}

		Ac.block<1,3>(i, 0) = (as[i] - cA) * wi;
		Bc.block<1,3>(i, 0) = (bs[i] - cB) * wi;
	}


	MatrixXd H = Ac.transpose() * Bc;


	JacobiSVD<MatrixXd> svd(H, ComputeThinU | ComputeThinV);

	R = svd.matrixV() * svd.matrixU().transpose();

	if(R.determinant() < 0) {
		MatrixXd X = MatrixXd::Identity(as.size(), bs.size());
		X(as.size() - 1,  bs.size() - 1) = R.determinant();
		R = svd.matrixV() * X * svd.matrixU().transpose();
	}

	t = -R*cA + cB;
}

}
