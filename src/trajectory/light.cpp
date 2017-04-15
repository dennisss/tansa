#include <tansa/trajectory.h>

namespace tansa {

int LightTrajectory::rgbiToInt(Color in, float i){
	int r_out = (int)(i*in.r*255);
	int g_out = (int)(i*in.g*255);
	int b_out = (int)(i*in.b*255);

	return ((b_out )| (g_out << 8) | (r_out << 16));
}


int LightTrajectory::evaluate(double t) {

	// Calculate percent intensities between start and end times
	double totalTimeChange = endTime - startTime;
	double requestedTimeChange = t - startTime;

	// Calculate the intensity percent from start time to passed in time
	double percentTotalChange = requestedTimeChange / totalTimeChange;
	double intensity = percentTotalChange * endIntensity + (1.0f - percentTotalChange)*startIntensity;
	Color out = startColor.interpolate_from_this(endColor, percentTotalChange);

	// Return the whole intensity at this time
	int ret = rgbiToInt(out, intensity);
	return ret;
}

}
