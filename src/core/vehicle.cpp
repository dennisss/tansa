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


// TODO: Move this to a better spot
// The ground control system represented by this program
static mavlink_system_t mavlink_system;

static mavlink_channel_t nextChannel = MAVLINK_COMM_0;


Vehicle::Vehicle() :
	lastControlInput(0,0,0),
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

int forwardfd = 0;
struct sockaddr_in forward_addr;

int Vehicle::connect(int lport, int rport, const char *laddr, const char *raddr) {

//	if((forwardfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
//		perror("cannot create socket");
//		return 1;
//	}

//	memset((char *)&forward_addr, 0, sizeof(client_addr));
//	forward_addr.sin_family = AF_INET;
//	forward_addr.sin_port = htons(14550);
//	inet_pton(AF_INET, "127.0.0.1", &forward_addr.sin_addr);


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
	pthread_join(thread, NULL);
	netfd = 0;
	thread = 0;
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


	Matrix3d m;
	m << 1, 0, 0,
		 0, -1, 0,
		 0, 0, -1;

	Vector3d pos_ned = enuToFromNed() * pos;
	Quaterniond orient_ned = Quaterniond(enuToFromNed()) * orient * Quaterniond(m);

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

	Vector3d accel_ned = enuToFromNed() * accel;

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

/*
			this->position = enuToFromNed() * Vector3d(lp.x, lp.y, lp.z);
			this->velocity = enuToFromNed() * Vector3d(lp.vx, lp.vy, lp.vz);
*/

//			printf("POS: %.2f %.2f %.2f\n", lp.x, lp.y, lp.z);

			break;

		case MAVLINK_MSG_ID_SYS_STATUS:
			// We mainly need to know the battery level
			mavlink_sys_status_t ss;
			mavlink_msg_sys_status_decode(msg, &ss);
			battery.voltage = ss.voltage_battery / 1000.0;
			battery.percent = ss.battery_remaining / 100.0;

			break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:

			break;


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
	}

}


void *vehicle_thread(void *arg) {
	Vehicle *v = (Vehicle *) arg;

	int res = 0;

	mavlink_message_t msg; // TODO: This should be set to all zeros initially
	mavlink_status_t status;

	memset(&msg, 0, sizeof(msg));

	char *buf = (char *) malloc(512);

	struct pollfd fds[1];
	int nfds = 1;

	// First descriptor for udp socket
	fds[0].fd = v->netfd;
	fds[0].events = POLLIN; // TODO: Set netfd to non-blocking and allow the OS to buffer the sendto if needed

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

//				sendto(forwardfd, buf, nread, 0, (struct sockaddr *) &forward_addr, addrlen);
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


	}

	return NULL;
}
