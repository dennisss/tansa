#include <tansa/trajectory.h>

namespace tansa {

int StrobeTrajectory::evaluate(double t) {

	// Calculate percent intensities between start and end times
	double totalTimeChange = endTime - startTime;
	double requestedTimeChange = t - startTime;
	if(requestedTimeChange < 0) {
		requestedTimeChange = 0;
	}

	// Calculate the intensity percent from start time to passed in time
	double percentTotalChange = requestedTimeChange / totalTimeChange;
	Color out = startColor.interpolate_from_this(endColor, percentTotalChange);
	double beatsPerSecond = endBeatsPerSecond * percentTotalChange + (1.0f - percentTotalChange)*startBeatsPerSecond;
	// Calculate how many full beats have passed
	int beatsPassed = (int)((t - startTime) * beatsPerSecond);
	int totalBeatsNeeded = (int)((endTime - startTime) * beatsPerSecond);
	
	// If odd or last, return the end intensity
	// If even, return the start intensity
	return (beatsPassed % 2 == 1 || beatsPassed == totalBeatsNeeded - 1) ?
		   rgbiToInt(out,(float)endIntensity) : rgbiToInt(out,(float)startIntensity);
}

}
