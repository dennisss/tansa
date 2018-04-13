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
	CameraPacketAdvertise = 1, /**< Idle cameras broadcast their id every second to let the host computer know to connect them; this also contains information as to the supported frame rates, resolutions, etc. */
	CameraPacketConfig = 2, /**< Sent from the host to a camera to (re-)initialize it. This also registers the host ip with the camera for data transmission */
	CameraPacketKeepalive = 3, /**< Must be sent every four seconds to each camera for it to continue sending back data */
	CameraPacketBlobs = 4,
	CameraPacketMJPEG = 5,
	CameraPacketVirtualSetState = 6 /** For a virtual camera, sets the position of the drone */

};

enum MocapCameraVendor {
	CameraVendorSelf = 1 /**< Cameras included in this repository */
};

/*
	These are formed by doing VENDOR*1000 + (Model #)
*/
enum MocapCameraModel {
	CameraModelVirtual = 1000, /**< Represents a simulated camera. The model number will correspond to the simulation position index  */
	CameraModelTracker0_1 = 1001, /**< A tracker v0.1 fitted with a Raspberry Pi Camera v1 */
	CameraModelTracker0_2 = 1002 /**< A tracker v0.2 fitted with a Raspberry Pi Camera v2 */
};


#define CAMERA_VENDOR_VIRTUAL 0
#define CAMERA_VENDOR_SELF 1
#define CAMERA_MODEL_SELF_RPICAM1 1
#define CAMERA_MODEL_SELF_RPICAM1 2



/**
 * Note: integers are little endian in the packets
 */
struct __attribute__((__packed__)) MocapCameraPacket {
	char magic[2]; /**< Should be 'TA' to distinguish the Tansa protocol */
	uint8_t type;
	//uint64_t timestamp; /**< Nanoseconds since the network coordinated epoch. For frame data, this is the most accurate measure of the end of frame capture (also exposure and shutter) */
	uint16_t size; /**< The length of the data section in bytes */
	char data[];
};

struct __attribute__((__packed__)) MocapCameraPacketAdvertisement {
	uint16_t model; /**<  */
	char serialNum[64]; /**< Should be a alphanumeric serial code upto 64 characters long */

	unsigned image_width;
	unsigned image_height;

	// TODO: should include allowable resolutions, allowable framerates (this could be framerate dependent)

};


struct __attribute__((__packed__)) MocapCameraPacketBlob {
	uint16_t x;
	uint16_t y;
	uint16_t radius;
};

struct __attribute__((__packed__)) MocapCameraPacketBlobs {
	uint32_t nblobs;
	MocapCameraPacketBlob blobs[];
};

struct __attribute__((__packed__)) MocapCameraPacketSetState {
	double position[3];
	double orientation[4];
};

enum MocapCameraMode {
	Off = 0, /**< Camera should be doing nothing. */
	Blob = 1,
	Video = 2 /**< Camera should stream back a video stream from the camera */
};

struct __attribute__((__packed__)) MocapCameraPacketConfig {
	uint8_t mode;
	uint8_t submode; /**< Mode specific options */
	uint8_t led_brightness; /**< If controllable, how bright the LEDs should be from 0-255 */
	uint8_t gain; /**< A camera defined gain between 0-255 which approximately maps to the ISO range of the camera sensor */
	uint32_t framerate; /**< FPS */
	uint32_t exposure; /**< In microseconds of open shutter time */
	// TODO: Control resolution (available ones should be queriable from the device)
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
