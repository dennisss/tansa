#include <tansa/trajectory.h>

double LightTrajectory::evaluate(double t) {

	// Calculate percent intensities between start and end times
	double totalTimeChange = endTime - startTime;
	double requestedTimeChange = t - startTime;
	assert(requestedTimeChange >= 0);
	assert(totalTimeChange > 0);

	// Calculate the intensity percent from start time to passed in time
	double percentTotalChange = requestedTimeChange / totalTimeChange;
	double requestedIntensityChange = percentTotalChange * (endIntensity - startIntensity);

	double intensity = std::abs(startIntensity + requestedIntensityChange);

	// Return the whole intensity at this time
	return intensity;
}