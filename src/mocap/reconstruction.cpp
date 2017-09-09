#include "reconstruction.h"
#include <tansa/vision/camera_model.h>
#include <tansa/vision/triangulation.h>

#include <iostream>

using namespace std;

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

	// Compute matches between all pairs of images
	// Keep track of which 'track' each point belongs to (may belong to multipl)
	// If there are conflicts, pick the point which minizes distance to epipolar lines
	//    and pick tracks that minimize total error of line displacement

	// When a point is triangulated, reproject it into every image and which for more supporting evidence

	// Use reprojection errors to

	// TODO: Once this is working, we can also track the motion of 2d points in the same camera between frames
	// TODO: Rigid body information could be fed back into this system in order to pre-estimate matches
	// TODO: When triangulating, we should use the dot radius information to further constrain the optimization (as we can infer distance based on relative scale)
	// TODO: If we know the radius of the markers used and we assume that they are always completely visible, then we can infer their position in 3d from a single 2d mearement of blob radius

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



	// Step 0: Init
	tracks.resize(0);
	trackLabels.resize(cameras.size());
	for(unsigned i = 0; i < cameras.size(); i++) {
		for(unsigned j = 0; j < trackLabels[i].size(); j++) { trackLabels[i][j] = -1; } // Reset any old stuff
		trackLabels[i].resize(frames[i].blobs.size(), -1);
	}

	// Step 1: Prepare tracks (being very picky about collisions)
	for(unsigned camI = 0; camI < cameras.size(); camI++) {
		for(unsigned camJ = camI + 1; camJ < cameras.size(); camJ++) {
			pairwiseMatch(camI, camJ);
		}
	}

	// Step 2: Triangulate all tracks
	for(unsigned i = 0; i < tracks.size(); i++) {
		vector<CameraModel> ms;
		vector<Vector2d> pts;
		for(unsigned j = 0; j < tracks[i].indices.size(); j++) {
			BlobIndex &idx = tracks[i].indices[j];
			ms.push_back( CameraModel::Default(cameras[idx.frameI].model) );

			ImageBlob b = frames[idx.frameI].blobs[idx.blobI];
			pts.push_back({ b.cx, b.cy });
		}

		Vector3d X = triangulatePoints(ms, pts);
		tracks[i].point = X;
		cout << "Triangulate: " << X.transpose() << " with " << ms.size() << " measurements" << endl;
	}
	// TODO

	// Step 2.5: Remove track points with high reprojection error

	// Step 3: Reproject the triangulations to the frames and find more matches
	// TODO

	// Step 4: Re-triangulate with all new points
	// TODO

	// Step 5: Reproject all points and reject all points based on mean squared error in reprojections
	// NOTE: This step should always be done at the very end


}


void PointReconstructor::pairwiseMatch(unsigned camI, unsigned camJ) {

	CameraModel m1 = CameraModel::Default(cameras[camI].model);
	CameraModel m2 = CameraModel::Default(cameras[camJ].model);


	Matrix<double, 3, 4> P1 = m1.projection();
	Matrix<double, 3, 4> P2 = m2.projection();
	Vector3d c1 = m1.position;
	Vector3d c2 = m2.position;

	//cout << "Models: " << cameras[camI].model << " " << cameras[camJ].model << endl;
	//cout << "Blobs: " << frames[camI].blobs.size() << " " << frames[camJ].blobs.size() << endl;


	// Compute fundamental matrix for this pair of cameras
	// Defined in 'Multiple View Geometry in Computer Vision' on page 244
	Matrix3d F = skewSymetric( projectPoint(P2, c1) ) * P2 * pseudoInverse(P1);

	// For each point in the first camera, find matches in the second camera
	for(unsigned i = 0; i < frames[camI].blobs.size(); i++) {

		int trackI = trackLabels[camI][i];
		if(trackI >= 0) {
			// Only allow going forward if this is the seed point of the track
			bool isTrackHead = tracks[trackI].indices[0].frameI == camI && tracks[trackI].indices[0].blobI == i;
			if(!isTrackHead) {
				// If this blob already belongs to a track, then we are not likely to match it to any blobs which weren't matched by the previous frames
				// This also gurantees thay we never pick more than 1 point from the same frame into a track
				continue;
			}
		}

		ImageBlob &b1 = frames[camI].blobs[i];

		// Homogeneous 2d point in first camera
		Vector3d x1(b1.cx, b1.cy, 1);

		// Line in the form ax + by + c = 0 in the second image
		Vector3d l = F*x1;


		// Now we perform a greedy search for the best possibly match
		int bestJ = -1;
		double bestDistance = 20.0; // This represents the maximum pixel distance required for a match
		double secondBestDistance = -1;
		for(int j = 0; j < frames[camJ].blobs.size(); j++) {

			int trackJ = trackLabels[camJ][j];
			if(trackJ >= 0) {
				continue;
			}

			ImageBlob &b2 = frames[camJ].blobs[j];

			// Homogeneous 2d point in second camera
			Vector3d x2(b2.cx, b2.cy, 1);

			// Distance from point in second camera to epipolar line
			double d = abs(l.dot(x2)) / l.segment<2>(0).norm();

			//cout << "x1: " << x1.transpose() << endl;
			//cout << "x2: " << x2.transpose() << endl;
			//cout << d << endl;
			if(d < bestDistance) { // TODO: Base this thershold dynamically on the radius of the dots
				if(bestJ >= 0) {
					secondBestDistance = bestDistance;
				}
				bestDistance = d;
				bestJ = j;
			}
		}

		if(bestJ >= 0) {
			// Reject matches based on Lowe's Ratio Test
			const double ratio = 0.7; // TODO: Tune this value
			if(secondBestDistance >= 0 && bestDistance >= ratio*secondBestDistance) {
				continue;
			}


			// Create a track if not already created
			if(trackI < 0) {
				tracks.push_back(Track());
				trackI = tracks.size() - 1;
				tracks[trackI].indices.push_back({ camI, i });
				trackLabels[camI][i] = trackI;
			}

			// Add the point from the second camera into the mix
			tracks[trackI].indices.push_back({ camJ, (unsigned) bestJ });
			trackLabels[camJ][bestJ] = trackI;
		}

	}

}





}
