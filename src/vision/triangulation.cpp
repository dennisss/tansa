#include <tansa/vision/triangulation.h>

#include <iostream>

using namespace std;

namespace tansa {

Vector3d triangulatePoints(const CameraModel &c1, const CameraModel &c2, const Vector2d &z1, const Vector2d &z2){

	Matrix<double, 3, 4> p1 = c1.projection(),
		p2 = c2.projection();

	Matrix4d A;
	A.row(0) = z1.x() * p1.row(2) - p1.row(0);
	A.row(1) = z1.y() * p1.row(2) - p1.row(1);
	A.row(2) = z2.x() * p2.row(2) - p2.row(0);
	A.row(3) = z2.y() * p2.row(2) - p2.row(1);

	Vector4d b = Vector4d::Zero();

	JacobiSVD<Matrix4d> svd(A, ComputeFullU | ComputeFullV);

	Vector4d S = svd.singularValues();
	Matrix4d V = svd.matrixV();

	Vector4d x = V.col(3);
	bool found = false;

//	if(!found){
//		return Vector3d(NAN, NAN, NAN);
//	}

	cout << A << endl;

	return x.segment<3>(0) / x.w();
}


}
