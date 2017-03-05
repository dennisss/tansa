#include <tansa/control.h>


/*
	Very very simple admittance controller for user interaction.

*/

namespace tansa {


AdmittanceController::AdmittanceController(Vehicle *v) : HoverController(v) {

}

void AdmittanceController::control(double t) {

	Vector3d v1 = this->getPoint();
	v1.z() = vehicle->state.position.z();


	double e = (v1 - vehicle->state.position).norm();

	double c = 0.7; // Distance away from reference at which 50% of reference is permitted

	e /= c; // e = c should be scaled to e = 1

	double alpha = 1 / (e*e + 1);  //0.9;

	// Handwavy clipping of no force case (we don't want to drift while no one is interacting with the drone)
	if(alpha > 0.7) {
		alpha = 1;
	}

	alpha = 0.90;

	if(e < 0.2)
		alpha = 1.0;

//	if(t > 20) {
//		printf("%.2f %.2f %.2f\n", t, alpha, e);
//		this->setPoint( this->getPoint() * alpha + vehicle->state.position * (1 - alpha) );
//	}

	HoverController::control(t);
}


};
