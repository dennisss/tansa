#include <tansa/control.h>

namespace tansa {

LightController::LightController(Vehicle *v) {
	this->vehicle = v;
	for(int i = 0; i < NUM_LIGHTS; i++){
		trajectories[i] = nullptr;
	}
	lightStates.resize(MAX_LIGHTS, 0);
}

void LightController::control(double t) {
	int values[NUM_LIGHTS];
	int i;

	if(t - this->lastTime < 0.033) { // Limit to around 30Hz
		return;
	}

	bool change = false;
	for(i = 0; i < NUM_LIGHTS; i++){
		if(trajectories[i] == nullptr) //TODO this is temporary need to figure out how to only do ones we have
			continue;
		values[i] = trajectories[i]->evaluate(t);
		if (abs(values[i] - lightStates[i]) >= EPSILON) {
			change = true;
			lightStates[i] = values[i];
		}
	}

	if(change) {
		this->lastTime = t;
		vehicle->set_lighting(lightStates);
	}
}

}
