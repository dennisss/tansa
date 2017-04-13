#include <tansa/control.h>

namespace tansa {

LightController::LightController(Vehicle *v) {
	this->vehicle = v;
	lightStates.resize(MAX_LIGHTS, 0);
}

void LightController::control(double t) {
	int values[NUM_LIGHTS];
	int i;
	
	for(i = 0; i < NUM_LIGHTS; i++){
		values[i] = trajectories[i]->evaluate(t);
		if (abs(values[i] - lightStates[i]) >= EPSILON) {
			lightStates[i] = values[i];
		}
	}
	
	vehicle->set_lighting(lightStates);
}

}
