#ifndef TANSA_NATNET_H_
#define TANSA_NATNET_H_
/*
	Definitions and types for the NatNet protocol
*/

#include <stdint.h>
#include <vector>
#include <memory>
#include <string>

namespace tansa {
namespace optitrack {

#define DEFAULT_MULTICAST_ADDRESS "239.255.42.99" // IANA, local network
#define DEFAULT_COMMAND_PORT 1510
#define DEFAULT_DATA_PORT 1511

#define MAX_NAMELENGTH 256 // maximum length for strings
#define MAX_PACKETSIZE 100000 // max size of packet (actual packet size is dynamic)


enum NatNetConnectionType {
	NatNetMulticast = 1,
	NatNetUnicast = 2
};


struct NatNetPacket {

	enum Type {
		Ping = 0,
		PingResponse = 1,
		Request = 2,
		Response = 3,
		RequestModelDef = 4,
		ModelDef = 5,
		RequestFrame = 6,
		Frame = 7,
		MessageString = 8,
		Disconnect = 9,
		UnrecognizedRequest = 100
	};

	uint16_t type;
	uint16_t size;
	char payload[MAX_PACKETSIZE];
} __attribute__((packed));

/**
 * A version defined as [major, minor, build, revision]
 */
typedef unsigned char NatNetVersion[4];

struct NatNetSender {
    char name[MAX_NAMELENGTH];
    NatNetVersion appVersion;
    NatNetVersion protocolVersion;
};



struct NatNetDescriptionData {};

struct NatNetDescription {

	enum Type {
		MarkerSet = 0,
		RigidBody,
		Skeleton,
		ForcePlate
	};

	~NatNetDescription() {
		if(data != NULL) delete data;
	}

    int32_t type;
	NatNetDescriptionData *data = NULL;
};

// All data descriptions for current session (as defined by host app)
struct NatNetDescriptions {
	static const int ID = 2;

	static void Parse(const char *ptr, const NatNetVersion v, NatNetDescriptions *desc);

	std::vector<NatNetDescription> arr;
};


struct NatNetMarker {
    int32_t id;
    float x, y, z;
    float size;
    int16_t params;

	bool isOccluded() const { return params & 0x01; }
	bool isPointCloudSolved() const { return params & 0x02; }
	bool isModelSolved() const { return params & 0x04; }
} __attribute__((packed));

struct NatNetPlainMarker {
	float x, y, z;
} __attribute__((packed));


struct NatNetMarkerSet {

	struct Description : NatNetDescriptionData {
	    char name[MAX_NAMELENGTH];
		std::vector<std::string> markerNames;
	};

    char name[MAX_NAMELENGTH];
	std::vector<NatNetPlainMarker> markers;
};


struct NatNetRigidBody {

	struct Description : NatNetDescriptionData {

		static const char *Parse(const char *ptr, const NatNetVersion v, NatNetRigidBody::Description *rb);

	    char name[MAX_NAMELENGTH];
	    int32_t id;
	    int32_t parent_id;
	    float offset_x, offset_y, offset_z; // offset position relative to parent
	};

	static const char *Parse(const char *ptr, const NatNetVersion v, NatNetRigidBody *rb);

    int32_t id;
    float x, y, z; // Position
    float qx, qy, qz, qw; // Orientation

	std::vector<NatNetPlainMarker> markerPositions;
	std::vector<int32_t> markerIds;
	std::vector<float> markerSizes;

    float meanError; // Mean measure-to-solve deviation
    int16_t params; // Host defined tracking flags

	// rigid body was successfully tracked in this frame
	bool isTrackingValid() const { return params & 0x01; }
};

// Skeleton Data
struct NatNetSkeleton {

	struct Description : NatNetDescriptionData {
	    char name[MAX_NAMELENGTH];
	    int32_t id;
	    std::vector<NatNetRigidBody::Description> rigidBodies;
	};

    int32_t id;
	std::vector<NatNetRigidBody> rigidBodies;
};

struct NatNetAnalogChannel {
	std::vector<float> framesValue;
};

struct NatNetForcePlate {

	struct Description : NatNetDescriptionData {
	    int32_t id; // used for order, and for identification in the data stream
	    char serialNum[128]; // for unique plate identification
	    float width; // plate physical width (manufacturer supplied)
	    float length; // plate physical length (manufacturer supplied)
	    float origin_x, origin_y, origin_z; // electrical center offset (from electrical center to geometric center-top of force plate) (manufacturer supplied)
	    float calMat[12][12]; // force plate calibration matrix (for raw analog voltage channel type only)
	    float corners[4][3]; // plate corners, in plate coordinates, clockwise from plate +x,+y (refer to C3D spec for details)
	    int32_t plateType; // force plate 'type' (refer to C3D spec for details)
	    int32_t channelDataType; // 0=Calibrated force data, 1=Raw analog voltages

		std::vector<char[MAX_NAMELENGTH]> channelNames;
	};


    int32_t id;

	std::vector<NatNetAnalogChannel> channels;

    int16_t params;
};


struct NatNetTimecode {
	// SMPTE format timecode
	uint32_t code;
	uint32_t subframe;

	void decode(int *hour, int *minute, int *second, int *frame, int *subframe);

	std::string to_string();

};

struct NatNetFrame {
	static const int ID = 1;

	typedef std::shared_ptr<NatNetFrame> Ptr;

	static const char *Parse(const char *ptr, const NatNetVersion v, NatNetFrame *frame);

	int32_t number;

	std::vector<NatNetMarkerSet> markerSets;
	std::vector<NatNetPlainMarker> otherMarkers;
	std::vector<NatNetRigidBody> rigidBodies;
	std::vector<NatNetSkeleton> skeletons;
	std::vector<NatNetMarker> labeledMarkers;
	std::vector<NatNetForcePlate> forcePlates;

	float latency;

	NatNetTimecode timecode;
    double timestamp;
    int16_t params;

	bool isRecording() const { return params & 0x01; }
	bool didTrackedModelsChange() const { return params & 0x02; }
};


}
}

#endif
