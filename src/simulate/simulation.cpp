
#include <tansa/simulate.h>

#include <iostream>

using namespace std;


namespace tansa {


Simulation::Simulation(World &world, WorldState &state) {
	this->world = world;
	this->state = state;
}

/*
void Simulation::setMotors(Vector4d m){
	for(int i = 0; i < 4; i++){


		// Physical constraint: limit the motor speeds to 0-1
		if(m[i] < 0){
			m[i] = 0;
			cerr << "motor limits! " << m.transpose() << endl;
		}
		else if(m[i] > 1){
			m[i] = 1;
			cerr << "motor limits! " << m.transpose() << endl;
		}


		// Motor step constraint
		int step = m[i] * 200;
		m[i] = ((double)step) / 200.0;
	}


	// Overall motor delay : ~2ms for usb serial latency and ~2ms max delay for 490Hz PWM
	motor_delay.push(pair<double, Vector4d>(state.time + 0.004, m));
	//this->motors = m;
}
*/


void Simulation::step() {

	double eps = 0.0001; // Update at 10kHz

	Time t(this->state.time.seconds() + eps);


	// Update states
	for(int i = 0; i < this->world.models.size(); i++) {
		this->world.models[i]->update(this->state.models[i], t);
	}

	this->state.time = t;

}


}
