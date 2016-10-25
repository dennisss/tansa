#ifndef TANSA_VEHICLE_H_
#define TANSA_VEHICLE_H_

#include "time.h"
#include "estimation.h"

#include <stdint.h>
#include <pthread.h>
#include <netinet/ip.h>

#include <mavlink.h>

#include <Eigen/Dense>
#include <string>

using namespace Eigen;
using namespace std;

#define MAV_CMD_BEACON MAV_CMD_USER_1


struct BatteryStatus {
	double voltage = -1;
	double percent = -1;

};


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

	/**
	 * Connect to the vehicle using the specified ports and ip address
	 *
	 * @param laddr the ip address of the local interface on which to listen for messages
	 * @param raddr the ip address of the remote vehicle. if null, then this will be determined by the first message received matching the given ports
	 */
	int connect(int lport = 14550, int rport = 14555, const char *laddr = NULL, const char *raddr = NULL);

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
	 * Land at the current position
	 */
	void land();


	/**
	 * Immediately kill motors.
	 * It may be wise to send this multiple times in case of packet loss
	 */
	void terminate();

	/**
	 * Sets the brightness of the LEDs on the drone
	 *
	 * The two parameters are floats representing brightness. They range from 0 (off) to 1.0 (full power)
	 */
	void set_lighting(float top, float bottom);

	/**
	 * Toggles the state of the active IR beacon. Once it is on, the
	 */
	void set_beacon(bool on);

	/**
	 * Fuses motion capture information into the current position estimate
	 */
	void mocap_update(const Vector3d &pos, const Quaterniond &orient, const Time &t);


	// TODO: Change these to use Point
	// By default, this will preserve the yaw
	void setpoint_pos(const Vector3d &p);

	void setpoint_accel(const Vector3d &a);

	// Should do something like waiting for a response
	void ping();


	// Connection state
	bool connected = false;
	bool armed = false;
	string mode;

	// Physical State : used for visualization and trajectory control
	State state;
	LinearComplementaryEstimator estimator;

	BatteryStatus battery;

private:

	friend void *vehicle_thread(void *arg);

	void handle_message(mavlink_message_t *msg);
	void handle_message_timesync(mavlink_message_t *msg);

	void send_message(mavlink_message_t *msg);

	void send_heartbeat();

	bool running = false; // Whether or not the server is running for this drone

	int netfd = 0;
	mavlink_channel_t channel;

	pthread_t thread = NULL;

	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;


	ControlInput lastControlInput;
	Time lastControlTime;


	Time lastHeartbeatReceived;

	// Time stuff
	void send_systime();
	void send_timesync(int64_t tc1, int64_t ts1);

	int64_t _time_offset;
	void smooth_time_offset(int64_t offset_ns);
	uint64_t sync_stamp(uint64_t usec);

	Time lastHeartbeatSent;
	Time lastTimesyncSent;
	Time lastSystimeSent;
	Time lastStateSent;
};

/**
 * Don't use this directly. This is used internally by the Vehicle class
 */
void *vehicle_thread(void *arg);


#endif
