#ifndef TANSA_VISION_SOLVE_PNP_H_
#define TANSA_VISION_SOLVE_PNP_H_

#include "camera_model.h"

#include <vector>

namespace tansa {

/**
 * Given 3d to 2d matches and the intrinsic parameters of a camera, this solves for the extrinsics
 */
bool solvePnP(const std::vector<Vector3d> &points, const std::vector<Vector2d> &observed, CameraModel *cam, bool useInitial = false);

}

#endif
