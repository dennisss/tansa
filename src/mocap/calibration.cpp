#include "calibration.h"

#include

namespace tansa {




/**
	UI
	- press button to mask existing objections
	- move wand,
	- press 'space' to collect a sample
	- press 'q' to finish this part
	- wait for calculation....
	- place wand on ground at the origin
	- press 'space' to capture it
	- program finishes processing and saves the data to a json calibration file
		- should consist of camera intrinsic parameters
			- focal lengths, center, active settings (frame rate, etc.)
			- tagged to the camera serial number etc. (TODO: On the raspberry pi the serial number is the cpu id)
		- should consist of camera positions and poses as well as calibration

	Input should be a collection of frame sets (M sets of images from all N cameras)
	- From these images we detect the calibration pattern
		- a rigid transfomation of the known wand pattern
		- we will assume that only the four calibration points are ever visible at a single time (no extraneous markers)
		- find the three colinear points
		- assert the third point is nicely tangential to them along the line projecting from the middle point
		- assert that the ratio between the points is reasonably close to the ideal constant-from-depth ratio
	- This will allow us to generate matches between frames
	- feed matches into bundle adjustment solver
		- this will give us ideal camera rotations and the positions (up to a scale factor)
		- NOTE: Because we know the geometry of the wand, we know much more than just the matches but rather also the 3d pose of the wand relative to each camera
			- We should try to use this
	- triangulate some of the points in the dataset to find what we think the size of the wand is
	- center the camera positions around their centroid and
	- place the wand at the origin on the
	- user presses enter
	- now we can perform a rigid alignment between the known wand geometry (in the canonical position) and the observed point cloud
	- use this transform to re-align the position and orientation of the camera group
		- NOTE: we can safely assume that the cameras will always be ABOVE ground

	TODO: We can test this with a simulated view of the usual wand with a random orientation and position w.r.t. the camera
...

 See http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment
 */
void MocapCameraCalibration::calibrateWanding(const std::vector<MocapCamera>& cameras, const std::vector<MocapCameraBlobsMsg> &samples) {



	// Collecting data
	

	// Currently the hardcoded pattern of the wand
	// Iti s
	vector<Vector3d> pattern = {
		{ -0.125, 0, 0 },
		{ 0, 0, 0 },
		{ 0.250, 0, 0 },
		{ 0, 0.2, 0 }
	};


	// Every 

	ceres::Problem problem;
	for (int i = 0; i < bal_problem.num_observations(); ++i) {

		// TODO: Once the wand pattern is found, we should be able to initialize the camera positions based on that

		// TODO: Initialize cameras to random positions in a constrained 10x10x10 meter box
		// Initialize camera parameters based on defaults

		ceres::CostFunction* cost_function =
		SnavelyReprojectionError::Create(
			bal_problem.observations()[2 * i + 0],
			bal_problem.observations()[2 * i + 1]);
			problem.AddResidualBlock(cost_function,
				NULL /* squared loss */,
				bal_problem.mutable_camera_for_observation(i),
				bal_problem.mutable_point_for_observation(i));
	}


	// Finally scale based on the known dimensions

}

bool MocapCameraCalibration::findWandProjection(const MocapCameraBlobsMsg &frame, const MocapCamera &camera, const std::vector<Vector3d> &pattern, std::vector<int> indices) {

	// We assert that there are only the pattern points in the frame
	// TODO: Evenetually we can just cluster the points and find the clusters with exactly 4 points

	// Iterate through all possible match combinations and solvePnP for all of them
	//vector<unsigned>
	//for(unsigned i = 0; i < )

	// n^n
	int ncombs = pattern.size();
	for(int i = 0; i < pattern.size() - 1; i++) {
		ncombs = ncombs*ncombs;
	}
	
	vector<int> comb(pattern.size(), 0);
	for(int iter = 0; iter < ncombs; iter++) {
		
		// Add 1 to it
		for(int i = comb.size() - 1; i >= 0; i--) {
			comb[i]++;
			if(comb[i] == comb.size()) { // Overflow
				comb[i] = 0;
			}
			else {
				break;
			}
		}
		
		// Filter to only the n! ones (distinct combinations)
		for(int i = 0; i < comb.size(); i++) {
			
		}
		
		
		
		
		
	}



	CameraModel model = CameraModel::Default(camera.model);

	vector<Vector2d> observed;
	for(int i = 0; i < frame.blobs.size(); i++) {
		observed.push(Vector2d(frame.blobs[i].cx, frame.blobs[i].cy));
		//observed.push_back(cam.projectPoint(pattern[i]));
		//cout << observed[i] << endl;
	}
	
	// Currently this assumes proper correspondense
	bool success = solvePnP(pattern, observed, &model);

	vector<Vector2d> reproj;
	for(int i = 0; i < pattern.size(); i++) {
		Vector2d re = cam.projectPoint(pattern[i]));
		
	}


}




}
