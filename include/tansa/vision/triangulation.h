#ifndef TANSA_VISION_TRIANGULATION_H_
#define TANSA_VISION_TRIANGULATION_H_

#include "camera_model.h"

#include <vector>

namespace tansa {

// TODO: Return booleans to indicate triangulation success

/**
 * Pairwise triangulation of points implemented via DLT
 */
Vector3d triangulatePoints(const CameraModel &c1, const CameraModel &c2, const Vector2d &z1, const Vector2d &z2);


/**
 * Also DLT based as in the appendix of Mierle Keir's thesis
 */
Vector3d triangulatePoints(const std::vector<CameraModel> &cs, const std::vector<Vector2d> &zs);


}



#endif
