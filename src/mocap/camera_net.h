#ifndef TANSA_MOCAP_CAMERA_PROTOCOL_H_
#define TANSA_MOCAP_CAMERA_PROTOCOL_H_

#include <stdint.h>



namespace tansa {

/*
	This file describes the data packets that are
*/



#define MOCAP_CAMERA_MASTER_DEFAULT_PORT 13000
#define MOCAP_CAMERA_NODE_DEFAULT_PORT 13001

enum MocapCameraPacketType {
	CameraPacketAdvertise = 1, // Idle cameras broadcast their id every second to let the host computer know to connect them; this also contains information as to the supported frame rates, resolutions, etc.
	CameraPacketConfig = 2, // Sent from the host to a camera to (re-)initialize it. This also registers the host ip with the camera for data transmission
	CameraPacketKeepalive = 3, // Must be sent every four seconds to each camera for it to continue sending back data
	CameraPacketBlobs = 4,
	CameraPacketMJPEG = 5

};

/**
 * Note: integers are little endian in the packets
 */
struct __attribute__((__packed__)) MocapCameraPacket {
	char magic[2]; /**< Should be 'TA' to distinguish the Tansa protocol */
	uint8_t type;
	uint16_t size; /**< The length of the data section in bytes */
	char data[];
};

struct MocapCameraPacketAdvertisement {
	unsigned vendor; /**< Who designed the camera */
	unsigned model; /**<  */
	char serialNum[64]; /**< Should be a alphanumeric serial code upto 64 characters long */

	// TODO: should include allowable resolutions, allowable framerates (this could be framerate dependent)

};


struct MocapCameraPacketBlob {
	uint16_t x;
	uint16_t y;
	uint16_t radius;
};

struct MocapCameraPacketBlobs {
	uint32_t nblobs;
	MocapCameraPacketBlob blobs[];
};


struct MocapCameraPacketConfig {


};

enum MocapCameraMode {
	Off = 0, /**< Camera should be  */
	Object = 1


};

struct MocapCameraPacketSettings {

	MocapCameraMode mode;

	uint32_t framerate;
	uint32_t exposure; /**< In microseconds of open shutter time */

	uint8_t led_brightness; /**< If controllable, how bright the LEDs should be from 0-255 */

	// Control resolution (available ones should be queriable from the device)

	/*
		TODO: Control ISO/brightness

		Set fixed AWB gains and fixed metering mode
		Will likely need to regulate
	*/

};


class MocapCameraPacketParser {
public:
	MocapCameraPacketParser();

	// Parses forward a single character
	bool parse(const char *c);

	MocapCameraPacket *get_packet() { return this->pkt; }

private:
	unsigned state = 0;
	MocapCameraPacket *pkt;


};

}


#endif
