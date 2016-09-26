#ifndef TANSA_VEHICLE_H_
#define TANSA_VEHICLE_H_

#include <stdint.h>
#include <pthread.h>
#include <netinet/ip.h>

#include <mavlink.h>

#include <Eigen/Dense>
#include <string>

using namespace Eigen;
using namespace std;

/**
 * Reprents a single remote quadcopter connected via UDP
 */
class Vehicle {

public:
	/**
	 * Initializes a new vehicle and
	 *
	 */
	Vehicle();


	int connect(int port = 14550);

	int disconnect();



	/**
	 * Changes the armed state of the drone
	 * Note: This action is asynchronous and completes once this.armed is changed
	 */
	void arm(bool armed);

	/**
	 * Set the PX4 flight mode of the drone i.e. 'position', 'manual', 'acro', 'offboard', etc.
	 */
	void set_mode(string mode);

	/**
	 * Sets the brightness of the LEDs on the drone
	 *
	 * The two parameters are floats representing brightness. They range from 0 (off) to 1.0 (full power) 
	 */
	void set_lighting(float top, float bottom);


	/**
	 * Fuses motion capture information into the current position estimate
	 */
	void mocap_update(uint64_t t, const Vector3d &pos, const Quaterniond &orient);


	// By default, this will preserve the yaw
	void setpoint_pos(const Vector3d &p);

	void setpoint_accel(const Vector3d &a);


	// Connection state
	bool connected = false;
	bool armed = false;
	string mode;

	// Physical State : used for visualization and trajectory control
	Vector3d position;
	Vector3d velocity;
	Quaterniond orientation;


private:

	friend void *vehicle_thread(void *arg);

	void handle_message(mavlink_message_t *msg);

	void send_message(mavlink_message_t *msg);

	void send_heartbeat();

	bool running = false; // Whether or not the server is running for this drone

	int netfd = 0;

	pthread_t thread = NULL;

	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;


};

/**
 * Don't use this directly. This is used internally by the Vehicle class
 */
void *vehicle_thread(void *arg);


#endif
