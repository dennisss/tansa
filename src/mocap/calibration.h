#ifndef TANSA_MOCAP_CALIBRATION_H_
#define TANSA_MOCAP_CALIBRATION_H_

#include "camera_pool.h"

#include <vector>

namespace tansa {

/**
	This handles the 'wanding' procedure for figuring out where the cameras are:

	1. Select all cameras currently connected as the calibration set

	2. Capture a frame set (a complete set : one from each camera in the same time step)
	3. Recognize wand pattern in each new frame set
		-> Use results to update completion counters in UI
		-> This is a 2D affine matching problem
	4. Repeat from 2

	5. User hits stop

	* At this point, all data has been collected

	6. Once all data has been collected, perform bundle adjustment across all valid data
		- Initialize instrinsic parameters with average parameters for the camera model
		-
		- Levenberg-Marquard algorithm
			- Use Ceres Solver for this

 */
class MocapCameraCalibration {
public:
	MocapCameraCalibration(const CameraPool &p);

	/**
	 * Step 1 in calibrating the system. Given frames from a wanding, this will figure out the relative positions and orientations of the cameras
	 */
	void calibrateWanding(const std::vector<MocapCameraBlobsMsg> &frames);

	/**
	 * Step 2 in calibrating. Given that the user put calibration markers on the ground,
	 */
	void calibrateGroundPlane(const MocapCameraBlobsMsg &frame);

private:

	/**
	 * For a single frame of the wanding routine, find the calibration wand
	 *
	 * This is essentially a bruteforce search over all possible combinations
	 * - For a wand with N points, there are N! possible matchings (for a 3 or 4 marker wand, this is ok)
	 * - For each possibility, solvePnP to get the pose of the camera w.r.t. the wand
	 * - Score it based on reprojection errors
	 * - Accept best match based on ratio test of reprojection errors
	 *
	 * @param frame
	 * @param pattern
	 * @param indices
	 */
	bool findWandProjection(const MocapCamera &camera, const MocapCameraBlobsMsg &frame, const std::vector<Vector3d> &pattern, std::vector<int> indices);

};


}

#endif
