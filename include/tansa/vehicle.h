#ifndef TANSA_VEHICLE_H
#define TANSA_VEHICLE_H

#include "time.h"
#include "estimation.h"
#include "trajectory.h"
#include "channel.h"
#include "data.h"

#include <stdint.h>
#include <pthread.h>
#include <netinet/ip.h>

#include <mavlink.h>

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <mutex>

using namespace Eigen;
using namespace std;

namespace tansa {

#define MAV_CMD_BEACON MAV_CMD_USER_1

#define MAV_CMD_RGBLED MAV_CMD_USER_2

struct BatteryStatus {
	static const int ID = 1;

	double voltage = -1;
	double percent = -1;

};

struct ActuatorOutputs {
	static const int ID = 2;

	vector<float> outputs;
};

struct TextMessage {
	static const int ID = 3;

	uint8_t severity;
	string text;
};


struct VehicleParameters {

	struct {
		Point p;
		Point i;
		Point d;
	} gains;

	double hoverPoint;

	double latency;
};

struct VehicleForwarder {
	int netfd;
	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;
};


/**
 * Reprents a single remote quadcopter connected via UDP
 */
class Vehicle : public Channel {

public:
	typedef std::shared_ptr<Vehicle> Ptr;

	/**
	 * Initializes a new vehicle and
	 *
	 */
	Vehicle();

	/**
	 * Connect to the vehicle using the specified ports and ip address
	 *
	 * @param lport the port on which we should wait for messages
	 * @param rport the port that is sending the messages (and should be sent outbound messages)
	 * @param laddr the ip address of the local interface on which to listen for messages
	 * @param raddr the ip address of the remote vehicle. if null, then this will be determined by the first message received matching the given ports
	 */
	int connect(int lport = 14550, int rport = 14555, const char *laddr = NULL, const char *raddr = NULL);

	int disconnect();


	/**
	 * Forward messages to another program
	 *
	 * @param lport the port on which we will listen for messages
	 * @param rport the port to which messages received from the vehicle will be sent
	 */
	int forward(int lport, int rport);


	bool read_params(string file);

	void write_params(string file);

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
	 *
	 * Note: This is deprecated
	 */
	void set_lighting(float top, float bottom);

	/**
	 * Sets the value of every lighting channel.
	 * There are a max of 7, where all non-specified ones default to 0. On the current reference design, channel 0 is the left downward LED, 1 will be the right. 2 will be on internal center led (which only supports solid binary mixes of R, B, and G channels)
	 */
	void set_lighting(const vector<int> &channels);

	/**
	 * Toggles the state of the active IR beacon. Once it is on, the
	 */
	void set_beacon(bool on);

	/**
	 * Fuses motion capture information into the current position estimate
	 */
	void mocap_update(const Vector3d &pos, const Quaterniond &orient, const Time &t);

	/**
	 * Computes the approximate state of the vehicle upon receiving a message sent right now
	 */
	ModelState arrival_state();

	// TODO: Change these to use Point
	// By default, this will preserve the yaw
	void setpoint_pos(const Vector3d &p);

	void setpoint_accel(const Vector3d &a, double yaw);

	void setpoint_attitude(const Quaterniond &att, double accel_z);

	void setpoint_zero();

	// Should do something like waiting for a response
	void ping();

	/**
	 * Request the value of an onboard parameter by name
	 */
	void param_read(const char *name);

	/**
	 * Starts gyro calibration onboard
	 */
	void calibrate_gyro();

	/**
	 * For sending simulated sensor data to the vehicle
	 */
	void hil_sensor(const Vector3d *acc, const Vector3d *gyro, const Vector3d *mag, const Time &t);

	void hil_gps(const Vector3d &latLongAlt, const Vector3d &vel, const Time &t);


	// Connection state
	bool connected = false;
	bool armed = false;
	bool tracking = false;
	string mode;

	// Physical State : used for visualization and trajectory control
	ModelState state;
	LinearComplementaryEstimator estimator;

	ControlInput lastControlInput;
	ControlInput lastRawControlInput;
	Time lastControlTime;

	double pingLatency;

	// State as observed by the onboard processor
	Time onboardPositionTime = Time(0, 0);
	ModelState onboardState;
	vector<double> lightState;

	BatteryStatus battery;

	VehicleParameters params;

	/**
	 * Parameters retrieved from the firmware running remotely
	 */
	DataObject onboardParams;


	Time lastRCTime = Time(0,0);


	bool get_message(TextMessage *m) {
		bool found = false;
		messagesLock.lock();
		if(messages.size() > 0) {
			*m = messages[0];
			messages.erase(messages.begin());
			found = true;
		}

		messagesLock.unlock();
		return found;
	}

private:

	friend void *vehicle_thread(void *arg);

	void handle_message(mavlink_message_t *msg);
	void handle_message_timesync(mavlink_message_t *msg);

	void send_message(mavlink_message_t *msg);

	void send_heartbeat();

	bool running = false; // Whether or not the server is running for this drone

	int netfd = 0;
	mavlink_channel_t channel;

	pthread_t thread = 0;

	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;

	vector<VehicleForwarder> forwarders;

	vector<TextMessage> messages;
	mutex messagesLock;

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
	Time lastTrackTime = Time(0,0); int ntracks = 0;
	Time lastPingTime = Time(0,0); int pingSeq = 0;
};

/**
 * Don't use this directly. This is used internally by the Vehicle class
 */
void *vehicle_thread(void *arg);


}


#endif
