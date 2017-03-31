
#include <tansa/simulate.h>

#include <pthread.h>

#include <iostream>

using namespace std;


namespace tansa {


Simulation *Simulation::Make() {

	World w;
	WorldState ws;

	DataObject desc = DataObject::LoadFile("config/models/x340.js");

	int n = 1;

	// Add all drones to the world
	for(int i = 0; i < n; i++) {
		MultirotorModel::Ptr m( new MultirotorModel(desc) );
		w.models.push_back(m);
		w.firmwares.push_back( shared_ptr<Firmware>( new Firmware(desc["firmware"], m) ) );
		w.firmwares[i]->start();
		State::Ptr s = m->defaultState();
		ws.models.push_back(s);
	}

	return new Simulation(w, ws);
}

Simulation::Simulation(World &world, WorldState &state) {
	this->world = world;
	this->state = state;
	this->thread = NULL;
}


/*
// TODO: Integrate this into gcs.cpp
void send_status_message(const WorldState &ws) {
	json jsonStatus;

	jsonStatus["type"] = "status";
	jsonStatus["time"] = ws.time.seconds();;

	json jsonVehicles = json::array();
	for(int i = 0; i < ws.models.size(); i++) {

		std::shared_ptr<MultirotorModelState> s = std::static_pointer_cast<MultirotorModelState>(ws.models[0]);


		json jsonVehicle;

		json jsonPosition = json::array();
		jsonPosition.push_back(s->position.x());
		jsonPosition.push_back(s->position.y());
		jsonPosition.push_back(s->position.z());
		jsonVehicle["position"] = jsonPosition;

		json jsonOrientation = json::array();
		jsonOrientation.push_back(s->orientation.w());
		jsonOrientation.push_back(s->orientation.x());
		jsonOrientation.push_back(s->orientation.y());
		jsonOrientation.push_back(s->orientation.z());
		jsonVehicle["orientation"] = jsonOrientation;

		json jsonMotors = json::array();
		jsonMotors.push_back(s->motors[0].throttle);
		jsonMotors.push_back(s->motors[1].throttle);
		jsonMotors.push_back(s->motors[2].throttle);
		jsonMotors.push_back(s->motors[3].throttle);
		jsonVehicle["motors"] = jsonMotors;

		jsonVehicles.push_back(jsonVehicle);
	}
	jsonStatus["vehicles"] = jsonVehicles;

	tansa::send_message(jsonStatus);
}
*/

extern Vector3d noiseVector(std::normal_distribution<> &dist, std::default_random_engine &gen);


Quaterniond noiseQuaternion(std::normal_distribution<> &dist, std::default_random_engine &gen) {
	Vector3d e = Vector3d( dist(gen), dist(gen), dist(gen) );
	return Quaterniond(AngleAxisd(e[0], Vector3d::UnitZ()) * AngleAxisd(e[1], Vector3d::UnitX()) * AngleAxisd(e[2], Vector3d::UnitZ()));
}

void *simulation_thread(void *arg) {
	printf("Simulation running in background");

	Simulation *s = (Simulation *) arg;


	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine gen = std::default_random_engine(seed);
	std::normal_distribution<> posNoise(0, 0.001);
	std::normal_distribution<> orientNoise(0, 0.0001);

	Time t = Time::realNow();

	unsigned i = 0;
	Rate r(10000);
	while(s->running) {

		Time tn = Time::realNow();


		s->world.firmwares[0]->update(s->state.models[0]);
		s->step(tn.since(t));

		i++;
		/*
		if(i % 500 == 0) { // 20Hz
			Time tn = Time::realNow();

			//cout << tn.since(t).seconds() << ": " << tn.since(t).seconds() - s->state.time.seconds() << endl;

			//cout << tn.since(t).seconds() << endl;
			//t = tn;
			//send_status_message(s->state);
			//cout << "T: " << sim.state.time.seconds() << endl;
		}
		*/

		if(i % 500 == 0) {
			//Time::setTime(s->state.time, 1.0);
			if(s->tracked) {

				std::shared_ptr<MultirotorModelState> ms = std::static_pointer_cast<MultirotorModelState>(s->state.models[0]);


				// TODO: Is this the right time to use? <- it should be the state time ( once we do all the setTime stuff properly)
				s->tracked->mocap_update(ms->position + noiseVector(posNoise, gen), noiseQuaternion(orientNoise, gen) * ms->orientation, tn);

			}
		}


		r.sleep();
	}

	return NULL;
}

void Simulation::start() {
	this->running = true;
	pthread_create(&this->thread, NULL, simulation_thread, this);
}

void Simulation::stop() {
	this->running = false;
	pthread_join(this->thread, NULL);
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


void Simulation::step(const Time &t) {
/*
	Time eps(0.0005); // Update at 2kHz

	Time t = this->state.time.add(eps);
*/

	// Update states
	for(int i = 0; i < this->world.models.size(); i++) {
		this->world.models[i]->update(this->state.models[i], t);
	}

	this->state.time = t;

}


}
