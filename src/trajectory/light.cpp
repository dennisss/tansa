#include <tansa/trajectory.h>

namespace tansa {

int LightTrajectory::rgbiToInt(Color in, float i){
	int r_out = (int)(i*in.r);
	int g_out = (int)(i*in.g);
	int b_out = (int)(i*in.b);

	return (r_out | g_out << 8 | b_out << 16);
}


int LightTrajectory::evaluate(double t) {

	// Calculate percent intensities between start and end times
	double totalTimeChange = endTime - startTime;
	double requestedTimeChange = t - startTime;

	// Calculate the intensity percent from start time to passed in time
	double percentTotalChange = requestedTimeChange / totalTimeChange;
	double intensity = percentTotalChange * startIntensity + (1.0f - percentTotalChange)*endIntensity;
	Color out = startColor.interpolate_from_this(endColor, percentTotalChange);

	// Return the whole intensity at this time
	return rgbiToInt(out, intensity);
}

}
