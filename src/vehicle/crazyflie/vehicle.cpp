
#include <tansa/trajectory.h>
#include "crtp_generic_setpoint.h"

/*

	For flow/loco:
	- Receive position
	- Send Pos+Vel setpoint


	For onboard mocap processing:
	- Send external position
	- Send Pos+Vel setpoint
	
	For offboard mocap processing:
	- Same but send 

*/

/*
	


*/

Vehicle::Vehicle(ConnectionPool *pool, const RadioUri &uri) {
	this->pool = pool;
	pool->
}

Vehicle::~Vehicle() {
	// 

}



void Vehicle::setpoint_state(const TrajectoryState &s) {
	setpoint_pos(s->position);
	setpoint_vel(s->velocity);
	// TODO: Feed forward acceleration
}


void Vehicle::setpoint_pos(const Vector3d &p) {
	crtp_message_t msg;
	msg.header = 0;
	msg.port = CRTP_PORT_SETPOINT_GENERIC;
	msg.size = sizeof(crtp_setpoint) + sizeof(crtp_setpoint_position);

	crtp_setpoint *sp = (crtp_setpoint *) msg.data;
	sp->type = CRTP_SETPOINT_POSITION;
	
	crtp_setpoint_position *psp = (crtp_setpoint_position *) sp->data;
	psp->x = p.x();
	psp->y = p.y();
	psp->z = p.z();

	send_message(&msg);
}

void Vehicle::setpoint_zero() {
	crtp_message_t msg;
	msg.header = 0;
	msg.port = CRTP_PORT_SETPOINT_GENERIC;
	msg.size = sizeof(crtp_setpoint);

	crtp_setpoint *sp = (crtp_setpoint *) msg.data;
	sp->type = CRTP_SETPOINT_STOP;
	
	send_message(&msg);
}


void Vehicle::send_message(crtp_message *msg) {
	// Copy into message queue for radio
}

// Ideally radios should support binding to specific

void Vehicle::handle_message(crtp_message *msg) {
	// Must call in from pool




	// TODO: Instead move logic completely to vehicle
	// But support smart handlers for specific 
	// TODO: A wildcard listener would still be useful for detecting new crazyflies if this is their first time connecting

	auto conn_state = radio->active_state;

	// Monitor connection status
	if(ackReceived) {
		this->success_count++;
		this->fail_count = 0;
		if(this->success_count > 20 && !this->connected) { // TODO: Needs to track per uri
			this->connected = true;
			printf("Connected!\n");
		}
	}
	else {
		this->success_count = 0;
		this->fail_count++;
		if(this->fail_count > 20 && this->connected){
			this->connected = false;
			printf("Lost connection!\n");
		}
	}

}