#ifndef TANSA_VISION_REPROJECTION_ERROR_H_
#define TANSA_VISION_REPROJECTION_ERROR_H_

#include <ceres/rotation.h>

namespace tansa {

/**
 * For bundle adjustment and other pose estimation purposes
 * NOTE: This assumes fixed image center and focal_x = focal_y
 */
class ReprojectionError {
public:

	ReprojectionError(double observed_x, double observed_y, double center_x, double center_y)
	 	: observed_x(observed_x), observed_y(observed_y), center_x(center_x), center_y(center_y) {

	}

	/**
	 *
	 * @param intrinsics [ focalLength, k1, k2 ]
	 * @param extrinsics [ R1, R2, R3, t1, t2, t3 ]
	 * @param point [ x, y, z ]
	 * @param residuals [ x, y ] <- in image coordinates
	 */
	template <typename T>
	bool operator()(const T* const intrinsics, const T* const extrinsics, const T* const point, T* residuals) const {

		// Transform the point into camera space
		T p[3];
		ceres::AngleAxisRotatePoint(camera, point, p); // Uses camera[0,1,2]
		p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5]; // Translate

		// Project into 2d
		T xp = p[0] / p[2];
		T yp = p[1] / p[2]; // TODO: if p[2] is zero, we need to fail


		// Calculating distortion based on k1 and k2 radial coefficients
		T r2 = xp*xp + yp*yp;
		T distortion = T(1.0) + r2*(intrinsics[1] + intrinsics[2]*r2);

		// Applying distortion
		T xpp = xp*distortion;
		T ypp = yp*distortion;

		// Compute final projected point position.
		const T& focal = intrinsics[0];
		T predicted_x = focal * xpp + T(center_x);
		T predicted_y = focal * ypp + T(center_y);

		// The error is the difference between the predicted and observed position.
		residuals[0] = predicted_x - T(observed_x);
		residuals[1] = predicted_y - T(observed_y);
		return true;
	}

private:
	double observed_x, observed_y, center_x, center_y;
};


}

#endif
