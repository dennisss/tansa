#include "reconstruction.h"
#include <tansa/vision/camera_model.h>
#include <tansa/vision/triangulation.h>

#include <iostream>

using namespace std;

/*
	Tansa will obide by the OpenCV camera model which has the camera looking down the +z space
	- All code outside of the OpenGL stuff will use the general pin hole camera model projection matrix for doing any


	For a physical camera modeled by:
	f_x 0 c_x
	0 f_y c_y
	0  0  1

	3*4
	4*1




	The equivalent OpenGL projection matrix is:
	Note: OpenGL cameras start looking at -z with +x to the right and +y up and the origin in the center of frame
	Note: For OpenCV/Computer vision, our camera looks in the +z with +x to the right and +y down

	2.0 * fx / width	0						1.0 - 2.0 * cx / width				0
	0					-2.0 * fy / height		2.0 * cy / height - 1.0				0
	0					0						(zfar + znear) / (znear - zfar)		2*(zfar*znear)/(znear-zfar)
	0					0						-1.0								0

	Note: The last two rows are equivalent to the usual OpenGL formulation


	Why the above matrix:
	- width maps to -1 to 1
		- so scaling from width to 2 is a factor of 1 / (width / 2) = 2 / width

	- flip y to be pointing up
		- it points down in opencv

	- Negate z to be looking down -z instead of +z
		- This is why the third row is negated from usual
*/

// We want to generate images out of OpenGL that are exactly how we'd
void cvToOpenGL() {



}


namespace tansa {


PointReconstructor::PointReconstructor(Context *ctx, MocapCameraPool *p) {
	this->pool = p;

	p->subscribe(ctx, &PointReconstructor::onCameraData, this);
	p->subscribe(ctx, &PointReconstructor::onCameraList, this);
}

// This is essentially meant for grabbing potentially out of order frames and
void PointReconstructor::onCameraData(const MocapCameraBlobsMsg *msg) {

	unsigned nReceived = 0;
	for(int i = 0; i < cameras.size(); i++) {

		if(msg->cameraId == cameras[i].id) {
			received[i] = true;
			frames[i] = *msg;
		}

		if(received[i]) {
			nReceived++;
		}
	}

	if(nReceived > 1 && nReceived == cameras.size()) {
		processFrames();

		for(int i = 0; i < received.size(); i++) {
			received[i] = false;
		}
	}

}

void PointReconstructor::onCameraList(const MocapCameraListMsg *msg) {
	// TODO: This may cause a corrupt frame set if it is in the process of running
	this->cameras = msg->cameras;
	this->frames.resize(cameras.size());
	this->received.resize(cameras.size(), false);
}

/**
 * Calculation of the pseudoinverse
 */
template<int N, int M>
Matrix<double, M, N>  pseudoInverse(const Matrix<double, N, M> &A) {
	//return (A.transpose() * A).inverse() * A.transpose();

	JacobiSVD<Matrix<double, N, M>> svd(A, ComputeFullV | ComputeFullU);

	double tol = 1.e-6; // choose your tolerance wisely!

	auto S = svd.singularValues();
	for(int i = 0; i < S.size(); i++) {
		if(S(i) > tol) {
			S(i) = 1.0 / S(i);
		}
		else {
			S(i) = 0;
		}
	}

	return svd.matrixV() * S.asDiagonal() * svd.matrixU().transpose();
}


Vector3d projectPoint(const Matrix<double, 3, 4> &proj, Vector3d pt) {
	Vector3d h = proj * Vector4d(pt.x(), pt.y(), pt.z(), 1);
	//return h;
	return Vector3d(h.x() / h.z(), h.y() / h.z(), 1);
}

Matrix3d skewSymetric(Vector3d v) {
	Matrix3d m;
	m << 0, -v.z(), v.y(),
		 v.z(), 0, -v.x(),
		 -v.y(), v.x(), 0;

	return m;
}


// To be called after a complete set of frames is available
void PointReconstructor::processFrames() {
	/*
		Fundamental matrix is a 3x3

		P1 projection matrix of first camera
		P2 projection matrix of second camera

		X is 3d point
		x1 is 2d point in first camera
		x2 is 2d point in second camera

		c1 is 3d camera center of

		x2' * F * x1 = 0
		F = skewSymetricMat( project(c1, P2) ) * P2 * pseudoInverse(P2)

	*/

	cout << "--------------" << endl;

	// Assuming only two cameras we
	int camI = 0;
	int camJ = 1;

	CameraModel m1 = CameraModel::Default(cameras[camI].model);
	CameraModel m2 = CameraModel::Default(cameras[camJ].model);


	Matrix<double, 3, 4> P1 = m1.projection();
	Matrix<double, 3, 4> P2 = m2.projection();
	Vector3d c1 = m1.position;
	Vector3d c2 = m2.position;

	cout << "Models: " << cameras[camI].model << " " << cameras[camJ].model << endl;
	cout << "Blobs: " << frames[camI].blobs.size() << " " << frames[camJ].blobs.size() << endl;


	// Compute fundamental matrix for this pair of cameras
	// Defined in 'Multiple View Geometry in Computer Vision' on page 244
	Matrix3d F = skewSymetric( projectPoint(P2, c1) ) * P2 * pseudoInverse(P1);

	//cout << F << endl;

	// TODO: For some reason these projected points as in a different order than the ones in the image data
	Vector3d v(0, 0.13, 0.04);
	cout << "V1: " << projectPoint(P1, v).transpose() << endl << "V2: " << projectPoint(P2, v).transpose() << endl;

	// For each point in the first camera, find matches in the second camera
	for(int i = 0; i < frames[camI].blobs.size(); i++) {

		ImageBlob &b1 = frames[camI].blobs[i];

		// Homogeneous 2d point in first camera
		Vector3d x1(b1.cx, b1.cy, 1);

		// Line in the form ax + by + c = 0 in the second image
		Vector3d l = F*x1;

		//cout << "L: " << l.transpose() << endl;

		for(int j = 0; j < frames[camJ].blobs.size(); j++) {
			ImageBlob &b2 = frames[camJ].blobs[j];

			// Homogeneous 2d point in second camera
			Vector3d x2(b2.cx, b2.cy, 1);

			//cout << "Ftest: " << (projectPoint(P2, v).transpose() * F * projectPoint(P1, v)).transpose()  << endl;
			//cout << "Ftest: " << (x2.transpose() * F * x1)  << endl;

			// Distance from point in second camera to epipolar line
			double d = abs(l.dot(x2)) / l.segment<2>(0).norm();

			cout << "x1: " << x1.transpose() << endl;
			cout << "x2: " << x2.transpose() << endl;
			cout << d << endl;
			if(d < 10) {
				Vector3d X = triangulatePoints(m1, m2, {b1.cx, b1.cy}, {b2.cx, b2.cy});
				cout << "Triangulate: " << X.transpose() << endl;
			}
		}

	}


	// TODO: Do the above for all image pairs (opposite pairs can be excluded as they are totally redundant)

	// Now perform triangulation incrementally picking best result at each step to reduce problem for next round


	// Possiblt

}




}
