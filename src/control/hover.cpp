#include <tansa/control.h>



HoverController::HoverController(Vehicle *v, const Point &p) {
	this->vehicle = v;
	this->point = p;
}

void HoverController::control(double t) {
	// This is already built into PX4, so we will let it handle staying in one spot
	// Precision over speed should considered when calibrating the PX4 position controller

	vehicle->setpoint_pos(point);
}

double HoverController::distance() {
	 return (point - vehicle->position).norm();
}
