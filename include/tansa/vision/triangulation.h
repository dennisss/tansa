#ifndef TANSA_VISION_TRIANGULATION_H_
#define TANSA_VISION_TRIANGULATION_H_

#include "camera_model.h"


namespace tansa {

/**
 * Pairwise triangulation of points implemented via DLT
 */
Vector3d triangulatePoints(const CameraModel &c1, const CameraModel &c2, const Vector2d &z1, const Vector2d &z2);


//Vector3d triangulatePoints(const vector<CameraModel> &cs, const vector<Vector2d> &zs);


}



#endif
