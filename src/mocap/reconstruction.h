#ifndef TANSA_MOCAP_RECONSTRUCTION_H_
#define TANSA_MOCAP_RECONSTRUCTION_H_

#include "camera_pool.h"
#include <tansa/core.h>

#include <vector>

namespace tansa {

/*
	This takes camera data, calibration parameters, and generates a 3d point cloud

	1. Receive camera data
		-> Use intrinsic params to undistort and normalize points
		-> Use extrinsic params to rectify the points
		-> Save these normalized points

	2. Keep receiving until we have one from each camera

	3. Perform n-camera epipolar matching -> for rectified points, this is just along a horizontal line
		-> Out of this, we get a list for each point of all other points which it could match up to

	4. All points are connected in a graph by whether or not they match together
		-> Triangulate one configuration
			-> Perform outlier rejection
		-> Accept strong triangulations

		-> We need to test all combinations of nodes in the
		-> Use noise tolerant SVD based triangulation



*/


/**
 * Published by the reconstructor with the results of the reconstruction process for one complete set of frames
 */
struct PointCloudMsg : Message {
	static const unsigned ID = 1;

	Time time;
	std::vector<Vector3d> points;
};


/**
 * Given an input source of
 */
class PointReconstructor : public Channel {
public:
	// TODO: We also need to provide:
	// - Database of Instrinsic and extrinsic parameters based on serial number
	PointReconstructor(Context *ctx, MocapCameraPool &p);


private:

	void onCameraData(MocapCameraBlobsMsg &msg);
	void onCameraList(MocapCameraListMsg &msg);

	std::vector<MocapCamera> cameras; /**< Copy of cameras we are working with */
	std::vector<MocapCameraBlobsMsg> buffer; /**< Recently received packets */

};



}

#endif
