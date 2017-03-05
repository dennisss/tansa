#include <tansa/simulate.h>

namespace tansa {


MultirotorModel::MultirotorModel(const DataObject &desc) : Model(desc) {

	imu = new IMU(desc["imu"]);
	gps = new GPS(desc["gps"]);

	battery = new Battery(desc["battery"]);


	double diameter = desc["diameter"];
	double r = diameter / 2.0;

	double l = r*cos(45 * (M_PI/180.0));

	json motorDesc = desc["motor"];

	// Motor 1
	motorDesc["position"] = { l, l, 0 };
	motorDesc["spin"] = COUNTERCLOCKWISE;
	motors.push_back(new Motor(DataObject(motorDesc)));

	// Motor 2
	motorDesc["position"] = { -l, -l, 0 };
	motorDesc["spin"] = COUNTERCLOCKWISE;
	motors.push_back(new Motor(DataObject(motorDesc)));

	// Motor 3
	motorDesc["position"] = { -l, l, 0 };
	motorDesc["spin"] = CLOCKWISE;
	motors.push_back(new Motor(DataObject(motorDesc)));

	// Motor 4
	motorDesc["position"] = { l, -l, 0 };
	motorDesc["spin"] = CLOCKWISE;
	motors.push_back(new Motor(DataObject(motorDesc)));

}

MultirotorModel::~MultirotorModel() {

}

State::Ptr MultirotorModel::defaultState() {
	MultirotorModelState *p = new MultirotorModelState();

	p->motors.resize(motors.size());

	return State::Ptr(p);
}


void MultirotorModel::update(State::Ptr _s, const Time &t) {

	std::shared_ptr<MultirotorModelState> s = std::static_pointer_cast<MultirotorModelState>(_s);


	// Evolve motor states and controls
	for(int i = 0; i < motors.size(); i++) {
		this->motors[i]->update(s->motors[i], t);

		// TODO: Move this else where
		//this->motors[i]->control(s->motors[i], firmware->currentActuatorOutputs[i]);
	}

	imu->update(s->imu, t);
	gps->update(s->gps, t);
	battery->update(s->battery, t);

	// Evolve battery state
	// TODO: This is coupled to the motors

	Model::update(_s, t);

	imu->observe(s->imu, *s);
	gps->observe(s->gps, *s);
}


Vector3d MultirotorModel::force(const State &_s) {
	const MultirotorModelState &s = (const MultirotorModelState &) _s;

	Vector3d f(0,0,0);
	for(int i = 0; i < motors.size(); i++) {
		f += motors[i]->force(s.motors[i]);
	}

	return f;
}

Vector3d MultirotorModel::torque(const State &_s) {
	const MultirotorModelState &s = (const MultirotorModelState &) _s;

	Vector3d t(0,0,0);
	for(int i = 0; i < motors.size(); i++) {
		t += motors[i]->torque(s.motors[i]);
	}

	return t;
}


}
