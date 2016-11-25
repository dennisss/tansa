#include <tansa/trajectory.h>


double StrobeTrajectory::evaluate(double t) {
	assert(t - startTime > 0); // TODO move to a constructor maybe

	// Calculate how many full beats have passed
	int secondsPassed = (t - startTime);
	int beatsPassed = secondsPassed * beatsPerSecond;
	int totalBeatsNeeded = (startTime - endTime) * beatsPerSecond;

	// If odd or last, return the end intensity
	// If even, return the start intensity
	return (beatsPassed % 2 == 1 || beatsPassed == totalBeatsNeeded - 1) ?
		   endIntensity : startIntensity;
}
