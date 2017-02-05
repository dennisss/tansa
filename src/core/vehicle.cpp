#include <tansa/vehicle.h>
#include "tf.h"

#include "../../lib/Firmware/src/modules/commander/px4_custom_mode.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>

namespace tansa {

// TODO: Move this to a better spot
// The ground control system represented by this program
static mavlink_system_t mavlink_system;

static mavlink_channel_t nextChannel = MAVLINK_COMM_0;


Vehicle::Vehicle() :
	lastControlInput(0,0,0),
	lastRawControlInput(0,0,0),
	lastControlTime(0,0),
	lastHeartbeatReceived(0,0),
	lastHeartbeatSent(0,0),
	lastTimesyncSent(0,0),
	lastSystimeSent(0,0),
	lastStateSent(0,0) {

	// TODO: Same as above
	mavlink_system.sysid = 255;
	mavlink_system.compid = MAV_COMP_ID_ALL;

	this->channel = nextChannel;
	nextChannel = (mavlink_channel_t) ((int)channel + 1);
}

int Vehicle::connect(int lport, int rport, const char *laddr, const char *raddr) {

	if((netfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket");
		return 1;
	}

	memset((char *)&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(lport);

	if (::bind(netfd, (struct sockaddr *) &server_addr, (socklen_t) sizeof(server_addr)) < 0) {
		perror("bind failed");
		close(netfd);
		netfd = 0;
		return 1;
	}

	int flags = fcntl(netfd, F_GETFL, 0);
	if(fcntl(netfd, F_SETFL, flags | O_NONBLOCK)) {
		perror("failed to make udp socket non-blocking");
		close(netfd);
		netfd = 0;
		return 1;
	}


	// By default send to 127.0.0.1:rport
	memset((char *)&client_addr, 0, sizeof(client_addr));
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(rport);
	inet_pton(AF_INET, "127.0.0.1", &client_addr.sin_addr);

	running = true;
	if(pthread_create(&thread, NULL, vehicle_thread, (void *) this) != 0) {
		running = false;
		return 1;
	}


	return 0;
}

int Vehicle::disconnect() {
	running = false;
	close(netfd);
	for(auto &f : forwarders) {
		close(f.netfd);
	}
	forwarders.resize(0);

	pthread_join(thread, NULL);
	netfd = 0;
	thread = 0;
	return 0;
}


int Vehicle::forward(int lport, int rport) {

	if(running) {
		printf("Cannot initialize a forwarder while already connected\n");
		return 1;
	}

	VehicleForwarder f;

	if((f.netfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("forward(): cannot create socket");
		return 1;
	}

	memset((char *)&f.client_addr, 0, sizeof(f.client_addr));
	f.client_addr.sin_family = AF_INET;
	f.client_addr.sin_port = htons(rport);
	inet_pton(AF_INET, "127.0.0.1", &f.client_addr.sin_addr);

	memset((char *)&f.server_addr, 0, sizeof(f.server_addr));
	f.server_addr.sin_family = AF_INET;
	f.server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	f.server_addr.sin_port = htons(lport);

	if (::bind(f.netfd, (struct sockaddr *) &f.server_addr, (socklen_t) sizeof(server_addr)) < 0) {
		perror("forward(): bind failed");
		close(netfd);
		return 1;
	}

	forwarders.push_back(f);

	return 0;
}

void Vehicle::arm(bool armed) {
	mavlink_message_t msg;
	mavlink_msg_command_int_pack(
		255, 0,
		&msg,
		1, 1,
		0,
		MAV_CMD_COMPONENT_ARM_DISARM,
		1,
		0,
		armed? 1 : 0,
		0,
		0,0,
		0,
		0,
		0
	);

	send_message(&msg);
}


void Vehicle::set_mode(string mode) {

	uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	if(this->armed) {
		base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	px4_custom_mode custom_mode;
	custom_mode.data = 0;
	custom_mode.main_mode = 0;

	if(mode == "manual")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
	else if(mode == "altitude")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
	else if(mode == "position")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
	else if(mode == "auto")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
	else if(mode == "acro")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
	else if(mode == "offboard")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
	else if(mode == "stabilized")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
	else if(mode == "rattitude")
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;


	mavlink_message_t msg;
	mavlink_msg_set_mode_pack(
		255, 0,
		&msg,
		1,
		base_mode,
		custom_mode.data
	);

	send_message(&msg);

}

void Vehicle::land() {
	// TODO: Use set_position_target_local_ned
	// From PX4: bool is_land_sp = (bool)(set_position_target_local_ned.type_mask & 0x2000);
	// See: http://discuss.px4.io/t/px4-hidden-flight-bitmasks/1371
}

void Vehicle::terminate() {

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(
		255, 0, channel,
		&msg,
		1, 1,
		MAV_CMD_DO_FLIGHTTERMINATION,
		0,
		2.0f, // kill motors in PX4 commander
		0, 0, 0, 0, 0, 0
	);

	send_message(&msg);

}


void Vehicle::mocap_update(const Vector3d &pos, const Quaterniond &orient, const Time &t) {

	// Update state (only compute velocity if initialized)
	Vector3d v(0,0,0);
	if(state.time.nanos() > 0) {
		Time dt = t.since(state.time);
		v = (pos - this->state.position) / dt.seconds();
	}


	// Forward predict upto mocap time
	// TODO: there may be a few overlapping control inputs so keep a list of control inputs
	this->estimator.predict(this->state, lastControlInput, t);

	this->estimator.correct(this->state, pos, v, t);


	Vector3d pos_ned = enuToFromNed() * pos;
	Quaterniond orient_ned = Quaterniond(enuToFromNed()) * orient * Quaterniond(baseToFromAirFrame());

	float q[4];
	q[0] = orient_ned.w();
	q[1] = orient_ned.x();
	q[2] = orient_ned.y();
	q[3] = orient_ned.z();


	// Send at 50Hz
	if(t.since(lastStateSent).seconds() < 0.02) {
		return;
	}

	lastStateSent = t;

	mavlink_message_t msg;
	mavlink_msg_att_pos_mocap_pack_chan(
		255, 0, channel, &msg,
		t.micros(),
		q,
		pos_ned.x(),
		pos_ned.y(),
		pos_ned.z()
	);

	send_message(&msg);


	// Update tracking status
	ntracks++;
	if(ntracks > 100) {
		this->tracking = true;
	}
	lastTrackTime = t;
}

State Vehicle::arrival_state() {
	if(params.latency == 0.0)
		return this->state;

	State newstate = this->state;

	Time eta(Time::now().seconds() + params.latency);

	// TODO: There may be multiple control inputs
	this->estimator.predict(newstate, lastControlInput, eta);

	return newstate;
}


void Vehicle::setpoint_pos(const Vector3d &pos) {

	lastControlInput = Vector3d(0,0,0);

	Vector3d pos_ned = enuToFromNed() * pos;

	mavlink_message_t msg;
	mavlink_msg_set_position_target_local_ned_pack_chan(
		255, 0, channel, // origin
		&msg,
		0,
		1, 1, // target
		MAV_FRAME_LOCAL_NED,
		0b0000101111111000, //
		pos_ned.x(),
		pos_ned.y(),
		pos_ned.z(),
		0,0,0,
		0,0,0,
		0,0
	);

	send_message(&msg);
}

void Vehicle::setpoint_accel(const Vector3d &accel) {

	lastControlInput = accel;
	// TODO: Once the state is forward predicted, use that time
	lastControlTime = Time::now();


	const double hover = params.hoverPoint;

	// Scale to -1 to 1 range and add hover point because PX4 doesn't take m s^-2 input but rather input proportional to thrust percentage
	Vector3d accel_normal = accel * (hover / GRAVITY_MS) + Vector3d(0, 0, hover);
	lastRawControlInput = accel_normal;

	Vector3d accel_ned = enuToFromNed() * accel_normal;

	mavlink_message_t msg;
	mavlink_msg_set_position_target_local_ned_pack_chan(
		255, 0, channel, // origin
		&msg,
		0,
		1, 1, // target
		MAV_FRAME_LOCAL_NED,
		0b0000110000111111, //
		0,0,0,
		0,0,0,
		accel_ned.x(),
		accel_ned.y(),
		accel_ned.z(),
		0,0
	);

	send_message(&msg);
}

void Vehicle::setpoint_zero() {
	this->setpoint_accel(Vector3d(0,0, -GRAVITY_MS));
}

void Vehicle::set_lighting(float top, float bottom) {
	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(
		255, 0, channel,
		&msg,
		1, 1,
		MAV_CMD_DO_SET_SERVO,
		0,
		top,
		bottom,
		0, 0, 0, 0, 0
	);

	send_message(&msg);
}

void Vehicle::set_beacon(bool on) {
	// TODO: This should wait for

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(
		255, 0, channel,
		&msg,
		1, 1,
		MAV_CMD_BEACON,
		1, // yes we want a confirmation
		on? 1 : 0,
		0, 0, 0, 0, 0, 0
	);

	send_message(&msg);
}

void Vehicle::set_rgb_lighting(int color) {

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(
		255, 0, channel,
		&msg,
		1, 1,
		MAV_CMD_RGBLED,
		0,
		color,
		0, 0, 0, 0, 0, 0
	);

	send_message(&msg);
}

void Vehicle::hil_sensor(const Vector3d *accel, const Vector3d *gyro, const Vector3d *mag, const Time &t) {

	Vector3d accel_ned, gyro_ned, mag_ned;
	if(accel != NULL) {
		accel_ned = enuToFromNed() * (*accel);
	}
	if(gyro != NULL) {
		gyro_ned = enuToFromNed() * (*gyro);
	}
	if(mag != NULL) {
		mag_ned = enuToFromNed() * (*mag);
	}


	mavlink_message_t msg;
	mavlink_msg_hil_sensor_pack_chan(
		255, 0, channel,
		&msg,
		t.micros(), // TODO: Check this
		accel_ned.x(),
		accel_ned.y(),
		accel_ned.z(),
		gyro_ned.x(),
		gyro_ned.y(),
		gyro_ned.z(),
		mag_ned.x(),
		mag_ned.y(),
		mag_ned.z(),
		0,
		0,
		0,
		0,
		// fields_updated
		(accel != NULL? 0b111 : 0) | (gyro != NULL? 0b111000 : 0) | (mag != NULL? 0b111000000 : 0)
	);

	send_message(&msg);

}

/*
// TODO: This should wait for a COMMAND_ACK message (should give a RESULT_ACCEPTED upon start of calibration)
// These src/modules/commander/calibration_messages.h are
void Vehicle::calibrate() {
	bool gyro = true;

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(
		255, 0, channel,
		&msg,
		1, 1,
		MAV_CMD_PREFLIGHT_CALIBRATION,
		1, // yes we want a confirmation
		gyro? 1 : 0,
		0, 0, 0, 0, 0, 0
	);

	send_message(&msg);
}
*/


void Vehicle::send_message(mavlink_message_t *msg) {
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	//mavlink_finalize_message_chan(msg, mavlink_system.sysid, mavlink_system.compid, this->channel, msg->len);
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	sendto(netfd, buf, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
}

void Vehicle::send_heartbeat() {
	mavlink_message_t msg;

	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_GCS;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

	uint8_t system_mode = MAV_MODE_PREFLIGHT;
	uint32_t custom_mode = 0;
	uint8_t system_state = MAV_STATE_STANDBY;

	// Pack the message
	mavlink_msg_heartbeat_pack_chan(mavlink_system.sysid, mavlink_system.compid, channel, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

	send_message(&msg);
}

void Vehicle::handle_message(mavlink_message_t *msg) {

	switch(msg->msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:

			mavlink_heartbeat_t hb;
			mavlink_msg_heartbeat_decode(msg, &hb);

			this->armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED? true: false;

			px4_custom_mode custom_mode;
			custom_mode.data = hb.custom_mode;

			switch(custom_mode.main_mode){
				case PX4_CUSTOM_MAIN_MODE_MANUAL: mode = "manual"; break;
				case PX4_CUSTOM_MAIN_MODE_ALTCTL: mode = "altitude"; break;
				case PX4_CUSTOM_MAIN_MODE_POSCTL: mode = "position"; break;
				case PX4_CUSTOM_MAIN_MODE_AUTO: mode = "auto"; break;
				case PX4_CUSTOM_MAIN_MODE_ACRO: mode = "acro"; break;
				case PX4_CUSTOM_MAIN_MODE_OFFBOARD: mode = "offboard"; break;
				case PX4_CUSTOM_MAIN_MODE_STABILIZED: mode = "stabilized"; break;
				case PX4_CUSTOM_MAIN_MODE_RATTITUDE: mode = "rattitude"; break;
				default: mode = "unknown";
			}

			lastHeartbeatReceived = Time::now();
			if(!this->connected) {
				printf("[Vehicle] Connected!\n");
				this->connected = true;
			}

			break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:

			mavlink_local_position_ned_t lp;
			mavlink_msg_local_position_ned_decode(msg, &lp);

			onboardState.position = enuToFromNed() * Vector3d(lp.x, lp.y, lp.z);
			onboardState.velocity = enuToFromNed() * Vector3d(lp.vx, lp.vy, lp.vz);

			break;

		case MAVLINK_MSG_ID_SYS_STATUS:
			// We mainly need to know the battery level
			mavlink_sys_status_t ss;
			mavlink_msg_sys_status_decode(msg, &ss);
			battery.voltage = ss.voltage_battery / 1000.0;
			battery.percent = ss.battery_remaining / 100.0;

			break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {

			mavlink_attitude_quaternion_t aq;
			mavlink_msg_attitude_quaternion_decode(msg, &aq);

			Quaterniond orient_ned(aq.q1, aq.q2, aq.q3, aq.q4);

			Quaterniond orient = Quaterniond(enuToFromNed()) * orient_ned * Quaterniond(baseToFromAirFrame());

			onboardState.orientation = orient;
			onboardState.time = Time::now();

			break;

		}
		case MAVLINK_MSG_ID_STATUSTEXT:

			// TODO: Also incorporate the severity
			mavlink_statustext_t st;
			mavlink_msg_statustext_decode(msg, &st);
			printf("%s\n", st.text);
			break;

		case MAVLINK_MSG_ID_SYSTEM_TIME: {
			mavlink_system_time_t st;
			mavlink_msg_system_time_decode(msg, &st);
			//printf("%llu %llu\n", st.time_unix_usec, Time::now().micros());

			// TODO: Simply log it to determine if the vehicle has adjusted its time correctly
			break;
		}

		case MAVLINK_MSG_ID_TIMESYNC:
			handle_message_timesync(msg);
			break;


		case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
			mavlink_hil_actuator_controls_t ac;
			mavlink_msg_hil_actuator_controls_decode(msg, &ac);

			// ac.controls is an array of floats from 0..1 with the motor outputs commanded
			ActuatorOutputs ao;
			for(int i = 0; i < 16; i++) {
				ao.outputs[i] = ac.controls[i];
			}

			this->publish(ao);
			break;
	}

}


void *vehicle_thread(void *arg) {
	Vehicle *v = (Vehicle *) arg;

	int res = 0;

	mavlink_message_t msg; memset(&msg, 0, sizeof(msg));
	mavlink_status_t status;


	char *buf = (char *) malloc(512);

	int nfds = 1 + v->forwarders.size();
	struct pollfd *fds = (struct pollfd *) malloc(sizeof(struct pollfd) * nfds);

	// First descriptor for udp socket
	fds[0].fd = v->netfd;
	fds[0].events = POLLIN; // TODO: Set netfd to non-blocking and allow the OS to buffer the sendto if needed

	// Other ones for forwarders
	for(int i = 0; i < v->forwarders.size(); i++) {
		fds[1 + i].fd = v->forwarders[i].netfd;
		fds[1 + i].events = POLLIN;
	}

	printf("Waiting for messages...\n");

	// Poll for messages
	while(v->running) {
		res = poll(fds, nfds, 500);
		if(res < 0) {
			// Error
		}
		else if(res == 0) {
			// timeout
		}
		else if(fds[0].revents & POLLIN) {
			struct sockaddr_in addr;
			socklen_t addrlen = sizeof(struct sockaddr_in);
			int nread = recvfrom(v->netfd, buf, 512, 0, (struct sockaddr *)&addr, &addrlen);

			// Register the client that is sending us messages
			v->client_addr.sin_port = addr.sin_port;
			v->client_addr.sin_addr = addr.sin_addr;

			if(nread > 0) {
				for(int i = 0; i < nread; i++) {
					// TODO: Pick comm channel based on the ip/port from which the data was received

					if(mavlink_parse_char(v->channel, buf[i], &msg, &status)) {

						v->handle_message(&msg);

					}
				}

				// Also send to all forwarding channels
				for(auto &f : v->forwarders) {
					sendto(f.netfd, buf, nread, 0, (struct sockaddr *) &f.client_addr, addrlen);
				}
			}
		}
		else { // Received data on one of the forwarder listeners

			struct sockaddr_in addr;
			socklen_t addrlen = sizeof(struct sockaddr_in);

			for(int i = 0; i < v->forwarders.size(); i++) {
				if(!(fds[1 + i].revents & POLLIN))
					continue;

				auto &f = v->forwarders[i];

				int nread = recvfrom(f.netfd, buf, 512, 0, (struct sockaddr *)&addr, &addrlen);

				// Register the client that is sending us messages
				f.client_addr.sin_port = addr.sin_port;
				f.client_addr.sin_addr = addr.sin_addr;

				// Send to vehicle
				sendto(v->netfd, buf, nread, 0, (struct sockaddr *)&v->client_addr, sizeof(v->client_addr));
			}

		}


		// Send all broadcasts: heartbeat every 1 second, system_time every 2 seconds and time sync every 2 seconds
		Time now = Time::now();


		if(v->connected && now.since(v->lastHeartbeatReceived).seconds() >= 2) {
			v->connected = false;
			printf("[Vehicle] Timed out!\n");
		}

		if(now.since(v->lastHeartbeatSent).seconds() >= 1) {
			v->send_heartbeat();
			v->lastHeartbeatSent = now;
		}
		if(now.since(v->lastSystimeSent).seconds() >= 2) {
			v->send_systime();
			v->lastSystimeSent = now;
		}
		if(now.since(v->lastTimesyncSent).seconds() >= 2) {
			v->send_timesync(0, now.nanos());
			v->lastTimesyncSent = now;
		}
		if(now.since(v->lastTrackTime).seconds() > 1) {
			v->tracking = false;
			v->ntracks = 0;
		}


	}

	free(buf);
	free(fds);

	return NULL;
}

}
