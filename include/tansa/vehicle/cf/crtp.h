
#pragma once

// Documentation available here: https://wiki.bitcraze.io/doc:crazyflie:crtp:index


#include <stdint.h>

#define CRTP_PORT_CONSOLE 0x00
#define CRTP_PORT_PARAM	0x02
#define CRTP_PORT_COMMANDER	0x03
#define CRTP_PORT_MEM 0x04
#define CRTP_PORT_LOG 0x05
#define CRTP_PORT_SETPOINT_GENERIC 0x07
#define CRTP_PORT_MAVLINK 0x08 // Non-standard port for transmitting mavlink messages

#define CRTP_PORT_PLATFORM 0x0D
#define CRTP_PORT_DEBUG 0x0E
#define CRTP_PORT_LINK 0x0F

#define CRTP_NULL(x) (((x).header & 0xf3) == 0xf3)

// 1 byte header + 31 bytes data = 32 (max ESB packet size)
// With the NRF51, this could be increased to ~250, but the Crazyradio PA uses a NRF24 which can't do this
#define CRTP_MAX_DATA_SIZE 31

typedef struct {
	uint8_t size; // Total size of this message, including the header (placed here to overlap with syslink length field)
	union {
		uint8_t header;
		struct {
			uint8_t channel : 2;
			uint8_t link : 2;
			uint8_t port : 4;
		};
	};

	uint8_t data[CRTP_MAX_DATA_SIZE];
} __attribute__((packed)) crtp_message_t;


typedef struct {
	float roll; // -20 to 20
	float pitch;
	float yaw; // -150 to 150
	uint16_t thrust;
} crtp_commander;
