#ifndef TANSA_MOCAP_CAMERA_MODEL_H_
#define TANSA_MOCAP_CAMERA_MODEL_H_

#include <tansa/core.h>

namespace tansa {

/**
 * Describes the parameters of your average physical camera and lens
 * THis contains both the intrinsic and extrinsic parameters right now
 */
class CameraModel {
public:

	/**
	 * Gets a default camera model (used by )
	 */
	static CameraModel Default(int id);

	Matrix<double, 3, 4> projection() const;

	unsigned width;
	unsigned height;
	double fx;
	double fy;
	double cx;
	double cy;

	Matrix3d rotation;
	Vector3d position;

	// TODO: Add in distortion coefficients
};

}

#endif
