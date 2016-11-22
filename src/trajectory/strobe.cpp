#include <tansa/trajectory.h>


double StrobeTrajectory::evaluate(double t) {
	assert(t - startTime > 0); // TODO move to a constructor maybe

	// Calculate how many full beats have passed
	int beatsPassed = (t - startTime) / secondsPerBeat;

	// If even or last, return the end intensity
	// If odd, return the start intensity
	return (beatsPassed % 2 == 0) ? endIntensity : startIntensity;
}
