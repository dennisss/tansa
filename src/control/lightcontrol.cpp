#include <tansa/control.h>

namespace tansa {


LightController::LightController(Vehicle *v) {
	this->vehicle = v;
}

void LightController::track(LightTrajectory *trajTop, LightTrajectory *trajBot) {
	this->trajectoryTop = trajTop;
	this->trajectoryBot = trajBot;
}

void LightController::control(double t) {

	// Evaluate trajectory
	double s1 = trajectoryTop->evaluate(t);
	double s2 = trajectoryBot->evaluate(t);

	// If either one has to be updated, update both.
	if (abs(s1 - currentIntensityTop) >= EPSILON
			|| abs(s2 - currentIntensityBot) >= EPSILON) {
		vehicle->set_lighting(s1, s2);
		currentIntensityTop = s1;
		currentIntensityBot = s2;
		printf("Light: %.2f and %.2f at %.2f\n", s1, s2, t);
	}
}

}
