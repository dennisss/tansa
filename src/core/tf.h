#ifndef TANSA_TF_H_
#define TANSA_TF_H_
/*
	Useful transforms
*/


#include <Eigen/Dense>


/*
	For converting our coordinate system to the flight controller
	ENU <-> NED
*/
inline Matrix3d enuToFromNed() {
	Matrix3d mat;
	mat << 0, 1, 0,
		   1, 0, 0,
		   0, 0, -1;
	return mat;
}

inline Matrix3d baseToFromAirFrame() {
	Matrix3d mat;
	mat << 1, 0, 0,
		   0, -1, 0,
		   0, 0, -1;
	return mat;
}


#endif
