#include <tansa/trajectory.h>

namespace tansa {

int LightTrajectory::WHITE = (0xff | 0xff << 8 | 0xff << 16);
double LightTrajectory::evaluate(double t) {

	// Calculate percent intensities between start and end times
	double percentTotalChange = getPercentDone(t);
	double requestedIntensityChange = percentTotalChange * (endIntensity - startIntensity);

	double intensity = std::abs(startIntensity + requestedIntensityChange);

	// Return the whole intensity at this time
	return intensity;
}

double LightTrajectory::getPercentDone(double time) {
	double timeSinceStart = time - startTime;
	double totalTimeChange = (endTime - startTime);
	assert(timeSinceStart >= 0);
	assert(totalTimeChange > 0);
	return timeSinceStart/ totalTimeChange;

}


int LightTrajectory::getColorAtTime(double time) {
	double percentage = getPercentDone(time);
	int ret;
	if(startColor == endColor)
		ret = startColor;
	else {
		int start_r =  ((startColor >> 16) & 0xff) / 0xff;
		int start_g = ((startColor >> 8) & 0xff) / 0xff;
		int start_b = ((startColor >> 0) & 0xff) / 0xff;

		int end_r =  ((endColor >> 16) & 0xff) / 0xff;
		int end_g = ((endColor >> 8) & 0xff) / 0xff;
		int end_b = ((endColor >> 0) & 0xff) / 0xff;

		int col_r = (int)((end_r - start_r) * percentage);
		int col_g = (int)((end_g - start_g) * percentage);
		int col_b = (int)((end_b - start_b) * percentage);

		ret = (col_b | (col_g << 8) | (col_r << 16));
	}
	return ret;
}
}
