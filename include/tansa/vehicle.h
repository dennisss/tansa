#ifndef TANSA_VEHICLE_H_
#define TANSA_VEHICLE_H_

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

	// TODO: Have separate ones for different controllers
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


class VehiclePool;

typedef int VehicleId;

/**
 * Reprents a single remote quadcopter connected via MAVLink
 */
class Vehicle : public Channel {

public:
	typedef std::shared_ptr<Vehicle> Ptr;

	~Vehicle();

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
	void set_lighting(const std::vector<int> &channels);

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


	// TODO: We should mutex lock most of these things, (or make them atomic)
	// Connection state
	bool connected = false;
	bool armed = false;
	bool tracking = false;
	std::string mode;

	// Physical State : used for visualization and trajectory control
	ModelState state;
	LinearComplementaryEstimator estimator;

	// TODO: Make this a vector of control inputs with a time for each of them (where the time is the time at which they will take effect aka the arrival time at the time of creation)
	ControlInput lastControlInput;
	ControlInput lastRawControlInput;
	Time lastControlTime;

	double pingLatency;

	bool overactuated = false; /**< Quick and dirty flag set if an attempt was made to exceed the actuator limits of the vehicle */

	// State as observed by the onboard processor
	Time onboardPositionTime = Time(0, 0);
	ModelState onboardState;
	std::vector<double> lightState;

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

	friend class VehiclePool;
	friend void *vehicle_pool_thread(void *arg);

	Vehicle(VehiclePool *pool, VehicleId id);

	void handle_message(mavlink_message_t *msg);
	void handle_message_timesync(mavlink_message_t *msg);
	void cycle(); /**< Frequently called by the connection handler to check for time variant events  */

	void send_message(mavlink_message_t *msg);

	void send_heartbeat();

	VehiclePool *_pool;
	VehicleId _id;

	mavlink_channel_t channel;


	std::vector<VehicleForwarder> forwarders;

	std::vector<TextMessage> messages;
	std::mutex messagesLock;

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


struct VehiclePoolConnection {
	VehicleId id;
	Vehicle::Ptr vehicle;
	struct sockaddr_in addr;
	Time lastMessage;
};

/**
 * A group of Vehicles sharing a single physical connection
 * This is the class which abstracts away the direct vehicle communication layer (udp, usb vs serial etc.)
 * This can be thought of as an object which listens on a single udp port for drones on different ips
 */
class VehiclePool {
public:
	typedef std::shared_ptr<VehiclePool> Ptr;


	VehiclePool(/*VehicleIdScheme idMaker */);


	/**
	 * I guess this would connect to all vehicles in
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
	 * Gets a reference to a vehicle by id.
	 * If it is not available, it will return an instance bound to this pool, but not currently connected
	 */
	Vehicle::Ptr get(VehicleId id);

	/**
	 * Get all vehicles that have been discovered on this pool and are currently connected
	 */
	std::vector<Vehicle::Ptr> available();



	/**
	 * Restricts the Vehicles in this bool to only originate from ips matching a single subnet
	 * This is used with simulation mode to restrict access to only local instances and not use real vehicles accidently
	 */
	void set_subnet(const char *ip, const char *mask);


private:

	friend class Vehicle;
	friend void *vehicle_pool_thread(void *arg);

	void send_message(VehicleId id, mavlink_message_t *msg);

	// TODO: We should alternatively support using a single mavlink channel and distinguishing by the
	VehicleId get_id(const struct sockaddr_in &addr); /**< Gets the id of the vehicle from the remote address */


	// TODO: Most of these will be going to the pool class
	bool running = false; // Whether or not the server is running for this drone
	int netfd = 0;
	pthread_t thread = 0;

	VehiclePool::Ptr self; /**< Used to create vehicles */

	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;

	// TODO: Access to this needs to be mutex locked
	// Maybe maintain a second copy in the vehicle_pool thread and only update the main one if a lock can be obtained quickly (although i'd need to handle bidirectional changes as well (so need a merge))
	std::mutex connectionsLock;
	std::map<VehicleId, VehiclePoolConnection> connections; /**< All vehicles detected in the pool organized by ip address */


};

/**
 * Don't use this directly. This is used internally by the Vehicle class
 */
void *vehicle_pool_thread(void *arg);



}


#endif
