#ifndef TANSA_MOCAP_CAMERA_PROTOCOL_H_
#define TANSA_MOCAP_CAMERA_PROTOCOL_H_

namespace tansa {

/*
	This file describes the data packets that are
*/


#include <stdint.h>


enum CameraPacketType {
	CameraPacketAdvertise = 1, // Idle cameras broadcast their id every second to let the host computer know to connect them; this also contains information as to the supported frame rates, resolutions, etc.
	CameraFrameConfig = 2, // Sent from the host to a camera to (re-)initialize it. This also registers the host ip with the camera for data transmission
	CameraPacketKeepalive = 3, // Must be sent every four seconds to each camera for it to continue sending back data
	CameraPacketBlob = 4,
	CameraPacketMJPEG = 5

};

struct __attribute__((__packed__)) CameraPacket {
	char magic[2]; /**< Should be 'TA' to distinguish the Tansa protocol */
	uint8_t type;
	uint16_t size; /**< The length of the data section in bytes */
	char data[];
};

struct CameraPacketBlob {
	uint16_t x;
	uint16_t y;
	uint16_t radius;
}

struct CameraPacketBlobData {
	uint32_t nblobs;
	CameraPacketBlob blobs[];
};


struct CameraPacketConfig {


};



class CameraNetworkNode {
public:

	void start(int lport, int rport);


	/**
	 * Called on a camera when 
	 */
	virtual void onTimeout();


};






}


#endif
