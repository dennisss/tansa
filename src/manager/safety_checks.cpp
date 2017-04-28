#include "safety_checks.h"


namespace tansa {


// TODO: Verify the long term stability of these checks

bool SafetyChecksHelper::check() {

	// TODO: Also check standard deviation of the position and orientation via mocap data

	// TODO: Move these checks to manager/safety_checks.cpp
	Vector3d vec = vehicle->state.orientation.toRotationMatrix() * Vector3d(0, 0, 1);
	if(vec.z() < 0) {
		printf("At least one drone upside down\n");
		return;
	}
	else if(vec.z() < 0.5) {
		printf("At least one drone tilted >45 degrees\n");
		return;
	}

	Time now = Time::now();

	if(now.since(vehicle->onboardState.time).seconds() > 4) {
		printf("Stale onboard orientation feedback\n");
		return;
	}

	if(now.since(vehicle->onboardPositionTime).seconds() > 4) {
		printf("Stale onboard position data\n");
		return;
	}

	// Simulation doesn't have RC input usally
	if(inRealLife && now.since(vehicle->lastRCTime).seconds() > 4) {
		printf("Some drones don't have RC\n");
		return;
	}

	// Compare our state estimate with the onboard state estimate
	// If these deviate by too much, then we probably have a
	Quaterniond d = vehicle->onboardState.orientation * vehicle->state.orientation.inverse();

	// TODO: lighten up this constraint or adjust the EKF more to use the correct angular covariance
	if(1.0 - fabs(d.w()) > 0.05) {
		printf("Onboard Pose Not Synced: Too large of a angular deviation\n");
		return;
	}


}





}
