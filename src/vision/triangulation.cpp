#include <tansa/vision/triangulation.h>


namespace tansa {

Vector3d triangulatePoints(const CameraModel &c1, const CameraModel &c2, const Vector2d &z1, const Vector2d &z2){

	Matrix<double, 3, 4> p1 = c1.projection(),
		p2 = c2.projection();

	Matrix4d A;
	A.row(0) = z1.x() * p1.row(2) - p1.row(0);
	A.row(1) = z1.y() * p1.row(2) - p1.row(1);
	A.row(2) = z2.x() * p2.row(2) - p2.row(0);
	A.row(3) = z2.y() * p2.row(2) - p2.row(1);


	JacobiSVD<Matrix4d> svd(A, ComputeFullU | ComputeFullV);
	Vector4d S = svd.singularValues(); // Sorted in descending order
	Matrix4d V = svd.matrixV();

	// TODO: Assert the rank of the nullspace is 1
	Vector4d x = V.col(3);

//	bool found = false;

//	if(!found){
//		return Vector3d(NAN, NAN, NAN);
//	}

	return x.segment<3>(0) / x.w();
}


Vector3d triangulatePoints(const std::vector<CameraModel> &cs, const std::vector<Vector2d> &zs) {

	if(cs.size() == 2) {
		return triangulatePoints(cs[0], cs[1], zs[0], zs[1]);
	}

	MatrixXd A = MatrixXd::Zero(3*cs.size(), 4 + cs.size());
	for(unsigned i = 0; i < cs.size(); i++) {
		A.block<3, 4>(3*i, 0) = cs[i].projection();
		A.block<3, 1>(3*i, 4 + i) = Vector3d(zs[i].x(), zs[i].y(), 1);
	}

	JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
	MatrixXd S = svd.singularValues(); // Sorted in descending order
	MatrixXd V = svd.matrixV();

	// TODO: Assert the rank of the nullspace is 1
	Vector4d x = V.col(V.cols() - 1);

	return x.segment<3>(0) / x.w();
}


}
