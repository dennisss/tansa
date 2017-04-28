#include <tansa/model.h>

/*
	Motor/Propeller/ESC
	-----

	Parameters:
	- kV
	- ESR
	- spinning direction (clockwise or counter-cw)
	- Attached propeller
		- Radius/diameter
		-

	State
	- t
	- speed
	- commanded thrust as PWM between 1000 and 2000

	Model



	Although not entirely accurate, we will assume that input is proportional to thrust
	So, input is proportional to w^2


	http://www.dronetrest.com/t/brushless-motors-how-they-work-and-what-the-numbers-mean/564
	Power loses (in watts):
	Copper lose =
	Iron lose = V * I_0

*/

// dx / dt = (1 / tau) (target(t) - x(t))

namespace tansa {


Motor::Motor(const DataObject &desc) {

	timeConstantUp = desc["time_constant_up"];
	timeConstantDown = desc["time_constant_down"];

	thrustCoeff = desc["thrust_coefficient"];
	torqueCoeff = desc["torque_coefficient"];

	spin = desc["spin"];

	position = desc["position"].matrix<3, 1>();

}



void Motor::update(MotorState &s, const Time &t) {

	s.throttle = s.commandedThrottle;


	double dt = s.time.since(t).seconds();

	double tc;
	if(s.commandedThrottle > s.throttle) {
		tc = this->timeConstantUp;
	}
	else {
		tc = this->timeConstantDown;
	}
/*
	double a = exp(-dt / tc);
	double b = 1.0 - a;

	s.speed = a*s.speed + b*s.controlSpeed;

	// TODO:
	// TODO: Instead, use the exact integration of the system
	s.throttle = s.throttle + ((s.commandedThrottle - s.throttle) / tc) * dt;
	*/
}



// Set the current desired
void Motor::control(MotorState &s, double throttle) {

	// TODO: Discretive to pwm values

	s.commandedThrottle = throttle;

}


Vector3d Motor::force(const MotorState &s) {
	// thrust_coefficient * (speed ^ 2)
	// TODO: Rotate by orientation
	return Vector3d(0, 0, this->thrustCoeff * s.throttle); //(s.speed*s.speed));

	double t = s.throttle;
	double t2 = t*t;

	return Vector3d(0, 0, 0.22846991*t2 + 0.10695395*t) * 9.81;
}

Vector3d Motor::torque(const MotorState &s) {
	// Torque produced by thrust and moments due to propeller drag
	return this->position.cross(this->force(s))
		+ Vector3d(0, 0, -this->spin * this->thrustCoeff * 0.016 * s.throttle); //(s.speed*s.speed));
}


}
