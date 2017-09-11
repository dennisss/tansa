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

	// TODO: Add in distortion?
	inline Vector2d projectPoint(const Vector3d &pt) {
		Vector3d h = this->projection() * Vector4d(pt.x(), pt.y(), pt.z(), 1);
		//return h;
		return Vector2d(h.x() / h.z(), h.y() / h.z());
	}

	unsigned width;
	unsigned height;
	double fx;
	double fy;
	double cx;
	double cy;
	double k1 = 0;
	double k2 = 0;

	Matrix3d rotation;
	Vector3d position;

};


// TODO: Find a better place for this

inline Vector3d toRodrigues(const Matrix3d &R) {
	AngleAxisd aa(R);

	Vector3d axis = aa.axis();
	double angle = aa.angle();
	if(angle < 0) {
		angle = -angle;
		axis = -axis;
	}

	return angle*axis;
}

inline Matrix3d fromRodrigues(const Vector3d &v) {
	double angle = v.norm();
	return AngleAxisd(angle, v / angle).toRotationMatrix();
}


}

#endif
