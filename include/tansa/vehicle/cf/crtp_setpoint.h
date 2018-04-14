/*
	Generic setpoint commands
	- setpoints accumalate, so giving two orthogonal setpoints will combine the effects of both (i.e. position + velocity)

	Decoded by the firmware here: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c

	Documented here:
	https://wiki.bitcraze.io/doc:crazyflie:crtp:generic_setpoint

	TODO: Compile Crazyflie firmware with  'make PLATFORM=CF2 CONTROLLER=mellinger'
*/

#define CRTP_SETPOINT_STOP 0 // NOTE: this one has an empty payload
#define CRTP_SETPOINT_VELOCITY 1
#define CRTP_SETPOINT_ZDIST 2
#define CRTP_SETPOINT_CPPM 3
#define CRTP_SETPOINT_ALTHOLD 4
#define CRTP_SETPOINT_HOVER 5
#define CRTP_SETPOINT_FULL 6
#define CRTP_SETPOINT_POSITION 7


typedef struct {
	uint8_t type;
	uint8_t data[];
} __attribute__((packed)) crtp_setpoint;


typedef struct {
	float vx;
	float vy;
	float vz;
	float vyaw;
} crtp_setpoint_velocity;

typedef struct {
	float x;
	float y;
	float z;
	float yaw;
} ctrp_setpoint_position;
