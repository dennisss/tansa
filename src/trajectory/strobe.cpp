#include <tansa/trajectory.h>

namespace tansa {

double StrobeTrajectory::evaluate(double t) {
	assert(t - startTime > 0); // TODO move to a constructor maybe

	// Calculate how many full beats have passed
	int beatsPassed = (t - startTime) * beatsPerSecond;
	int totalBeatsNeeded = (endTime - startTime) * beatsPerSecond;

	// If odd or last, return the end intensity
	// If even, return the start intensity
	return (beatsPassed % 2 == 1 || beatsPassed == totalBeatsNeeded - 1) ?
		   endIntensity : startIntensity;
}

}
