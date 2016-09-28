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
mavlink_system_t mavlink_system;


Vehicle::Vehicle() {

	// TODO: Same as above
	mavlink_system.sysid = 255;
	mavlink_system.compid = MAV_COMP_ID_ALL;

}

int Vehicle::connect(int port) {
	if((netfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket");
		return 1;
	}

	memset((char *)&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(port);

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


	// By default send to 127.0.0.1:14555
	memset((char *)&client_addr, 0, sizeof(client_addr));
	client_addr.sin_family = AF_INET;
	client_addr.sin_port = htons(14555);
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
	thread = NULL;
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

// MAV_CMD_NAV_LAND_LOCAL
// Set first three params to 0
// We should also probably mark the home position
}


void Vehicle::mocap_update(const Vector3d &pos, const Quaterniond &orient, uint64_t t) {

	Vector3d pos_ned = enuToFromNed() * pos;
	Quaterniond orient_ned(enuToFromNed() * orient);

	float q[4];
	q[0] = orient_ned.w();
	q[1] = orient_ned.x();
	q[2] = orient_ned.y();
	q[3] = orient_ned.z();

	mavlink_message_t msg;
	mavlink_msg_att_pos_mocap_pack(
		255, 0, &msg,
		t,
		q,
		pos_ned.x(),
		pos_ned.y(),
		pos_ned.z()
	);

	send_message(&msg);
}


void Vehicle::setpoint_pos(const Vector3d &pos) {

	Vector3d pos_ned = enuToFromNed() * pos;

	mavlink_message_t msg;
	mavlink_msg_set_position_target_local_ned_pack(
		255, 0, // origin
		&msg,
		0,
		1, 1, // target
		MAV_FRAME_LOCAL_NED,
		0b0000111111111000, //
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

	Vector3d accel_ned = enuToFromNed() * accel;

	mavlink_message_t msg;
	mavlink_msg_set_position_target_local_ned_pack(
		255, 0, // origin
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
	mavlink_msg_command_long_pack(
		255, 0,
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
	mavlink_msg_command_long_pack(
		255, 0,
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
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

	send_message(&msg);
}

void Vehicle::handle_message(mavlink_message_t *msg) {

	switch(msg->msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:

			mavlink_heartbeat_t hb;
			mavlink_msg_heartbeat_decode(msg, &hb);

			this->armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED? true: false;
			this->connected = true;

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


			// TODO: Update last seen time and connected status

			printf("GOT HEARTBEAT\n");
			break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:

			mavlink_local_position_ned_t lp;
			mavlink_msg_local_position_ned_decode(msg, &lp);

			this->position = enuToFromNed() * Vector3d(lp.x, lp.y, lp.z);
			this->velocity = enuToFromNed() * Vector3d(lp.vx, lp.vy, lp.vz);

			//printf("POS: %.2f %.2f %.2f\n", position.x(), position.y(), position.z());

			break;

		case MAVLINK_MSG_ID_SYS_STATUS:
			// We mainly need to know the battery level

			break;

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:

			break;


		case MAVLINK_MSG_ID_STATUSTEXT:

			// TODO: Also incorporate the severity
			mavlink_statustext_t st;
			mavlink_msg_statustext_decode(msg, &st);
			printf("%s\n", st.text);
			break;

		// TODO: STATUSTEXT
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

					if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

						v->handle_message(&msg);

						// Packet received
						//printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
					}
				}
			}
		}


		// Send heartbeat at most every 0.5 seconds

	}

	return NULL;
}
