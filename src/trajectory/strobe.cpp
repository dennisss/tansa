#include <tansa/trajectory.h>

namespace tansa {

int StrobeTrajectory::evaluate(double t) {

	// Calculate percent intensities between start and end times
        double totalTimeChange = endTime - startTime;
        double requestedTimeChange = t - startTime;

        // Calculate the intensity percent from start time to passed in time
        double percentTotalChange = requestedTimeChange / totalTimeChange;
        double intensity = percentTotalChange * startIntensity + (1.0f - percentTotalChange)*endIntensity;
        Color out = startColor.interpolate_from_this(endColor, percentTotalChange);
	double beatsPerSecond = startBeatsPerSecond * percentTotalChange + (1.0f - percentTotalChange)*endBeatsPerSecond;
	// Calculate how many full beats have passed
	int beatsPassed = (t - startTime) * beatsPerSecond;
	int totalBeatsNeeded = (endTime - startTime) * beatsPerSecond;
	
	// If odd or last, return the end intensity
	// If even, return the start intensity
	return (beatsPassed % 2 == 1 || beatsPassed == totalBeatsNeeded - 1) ?
		   rgbiToInt(out,endIntensity) : rgbiToInt(out,startIntensity);
}

}
