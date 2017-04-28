#include <tansa/control.h>

namespace tansa {

LightController::LightController(Vehicle::Ptr v) {
	this->vehicle = v;
	for(int i = 0; i < NUM_LIGHTS; i++){
		trajectories[i] = nullptr;
	}
	lightStates.resize(MAX_LIGHTS, 0);
}

void LightController::control(double t) {
	int values[NUM_LIGHTS];
	int i;

	for(i = 0; i < NUM_LIGHTS; i++){
		if(trajectories[i] == nullptr) //TODO this is temporary need to figure out how to only do ones we have
			continue;
		values[i] = trajectories[i]->evaluate(t);
		if (abs(values[i] - lightStates[i]) >= EPSILON) {
			lightStates[i] = values[i];
		}
	}
	vehicle->set_lighting(lightStates);
}

}
