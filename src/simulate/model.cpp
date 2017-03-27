#include <tansa/model.h>

namespace tansa {

Model::Model(const DataObject &desc) {
	this->mass = desc["mass"];
	this->mInertia = desc["moments"].matrix<3,3>();
}

State::Ptr Model::defaultState() {
	return State::Ptr(new ModelState());
}

void Model::update(State::Ptr _s, const Time &t) {

	ModelState::Ptr s = std::static_pointer_cast<ModelState>(_s);

	double dt = t.since(s->time).seconds();

	// Unpack old state
	Vector3d x = s->position;
	Vector3d v = s->velocity;
	Matrix3d R = s->orientation.matrix();
	Vector3d w = s->angularVelocity;


	// Perform rigid body dynamics integration

	Vector3d f = this->force(*_s),
			 tau = this->torque(*_s);

	// Linear acceeration
	Vector3d a = Vector3d(0,0, -this->mass*GRAVITY_MS) + R * f;
	a /= this->mass;

	// Euler equation to get body angular acceleration
	Vector3d dw = this->mInertia.inverse() * (tau - w.cross(this->mInertia * w));

	// Quaternion derivative
	Quaterniond dq = s->orientation * Quaterniond(0, 0.5*w.x(), 0.5*w.y(), 0.5*w.z());


	// Perform the integration

	// Simple ground plane collision clipping
	// TODO: Instead, planes should apply a normal force equal to the
	if(s->position.z() <= 0 && a.z() < 0) {
		a.z() = 0; // Adding a normal force equivalent to the downward force
		x.z() = 0;
		v.z() = 0;
	}

	s->acceleration = a;
	s->velocity = v + a*dt;
	s->position = x + v*dt + 0.5*a*dt*dt;

	s->angularAcceleration = dw;
	s->angularVelocity = w + dw*dt;
	s->orientation = Quaterniond( s->orientation.coeffs() + dq.coeffs()*dt );
	s->orientation.normalize();

	s->time = t;
}


}
