#ifndef TANSA_BEACON_CALIBRATION_H_
#define TANSA_BEACON_CALIBRATION_H_

#include <tansa/core.h>
#include <vector>

namespace tansa {


class BeaconCalibration {
public:


	/**
	 * Given the distances between all pairs of beacons, figures out a valid positioning of the beacons
	 */
	bool estimate_positions(const MatrixXd &distances, std::vector<Vector3d> *out);

};


}

#endif
