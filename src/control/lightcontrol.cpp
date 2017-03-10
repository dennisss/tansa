#include <tansa/control.h>

namespace tansa {


LightController::LightController(Vehicle *v) {
	this->vehicle = v;
}

void LightController::track(LightTrajectory *trajectory) {
	this->trajectory = trajectory;
}

int LightController::calcFromIntensity(double intensity) {
	assert (intensity <= 1.01);
	int value = (int)std::floor(intensity*0xff);

	return ( value | value << 8 | value << 16);
}

void LightController::control(double t) {

	// Evaluate trajectory
	double s1 = trajectory->evaluate(t);

	// If either one has to be updated, update both.
	if (abs(s1 - currentIntensity) >= EPSILON){
		currentIntensity = s1;
		if(trajectory->isWhiteOnly) {
			vehicle->set_rgb_lighting(calcFromIntensity(currentIntensity));
			printf("Light: %.2f at %.2f\n", s1, t);
		} else {
			int color = trajectory->getColorAtTime(t);
			vehicle->set_rgb_lighting(color);
		}
	}
}

}
