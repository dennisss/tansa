#include <tansa/vision/camera_model.h>

namespace tansa {


// Parameters of the real camera we are trying to simulate (currently set to the nominal parameters of a RPi Camera v2)
#define DEFAULT_CAMERA_WIDTH 1640 // 1280
#define DEFAULT_CAMERA_HEIGHT 1232 // 720
#define DEFAULT_CAMERA_FX 1354
#define DEFAULT_CAMERA_FY 1354
#define DEFAULT_CAMERA_CX 820
#define DEFAULT_CAMERA_CY 616

CameraModel CameraModel::Default(int id) {

	float angle = ((float)id)*M_PI / 3.0;
	float radius = 0.5;
	float height = 0.4;

	float x = radius*cos(angle);
	float y = radius*sin(angle);


	// First convert to ENU
	// Make looking along +y
	Matrix3d R;
	R << 1, 0, 0,
		0, 0, 1,
		0, -1, 0;

	// Rotate about circle
	Matrix3d Rc( AngleAxisd((M_PI / 2) + angle, Vector3d::UnitZ()) );
	R = Rc * R;

	// Add declination
	Matrix3d Rd( AngleAxisd(-30 * (M_PI / 180.0), Rc*Vector3d::UnitX()) );
	R = Rd * R;



	CameraModel m;
	m.width = DEFAULT_CAMERA_WIDTH;
	m.height = DEFAULT_CAMERA_HEIGHT;
	m.fx = DEFAULT_CAMERA_FX;
	m.fy = DEFAULT_CAMERA_FY;
	m.cx = DEFAULT_CAMERA_CX;
	m.cy = DEFAULT_CAMERA_CY;
	m.k1 = 0;
	m.k2 = 0;
	m.rotation = R;
	m.position = {x, y, height};

	return m;
}


Matrix<double, 3, 4> CameraModel::projection() const {

	Matrix3d a;
	a << fx, 0, cx,
		 0, fy, cy,
		 0, 0, 1;

	Matrix<double, 3, 4> b;
	b.block<3,3>(0, 0) = rotation.transpose();
	b.block<3,1>(0, 3) = -rotation.transpose()*position;

	return a*b;
}


}
