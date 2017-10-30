/*
	Definitions for the protocol used between beacons using the UWB transciever
*/


#define BEACON_ERROR_OK 0
#define BEACON_ERROR_USAGE 1 /**< When a command was improperly called (typically over serial ) */
#define BEACON_ERROR_TIMEOUT 2 /**< When a packet has timed out  */

// These are used for simple
// TODO: I think 1 millisecond woul be fine
#define BEACON_RESPONSE_DELAY 1000 // For ping messages, How many time steps should be between receiving a message and responding to it ()

#define BEACON_PACKET_STAT 1 // Ask a
#define BEACON_PACKET_IDENTIFY 2 // Asks a beacon to visually identify itself by flashing its LED
#define BEACON_PACKET_PING 3 // Sent from one node to another to request an
#define BEACON_PACKET_BROADCAST 4 // Sends the current clock time : Used for TDoA anchor packets
#define BEACON_PACKET_PROXY_PING 5 // Request one beacon to ping another beacon
#define BEACON_PACKET_CONFIG 6 // Used to set some va
#define BEACON_PACKET_ACK (1 << 7) // The upper bit in any message means it is an acknowledgement of the main message

#define BEACON_ADDR_BROADCAST 0 /**< Use this address to send to everyone */
// Addresses 1-10 are reserved for anchor ids
// Addresses 11-14 are reserved for other two way communication nodes
#define BEACON_ADDR_COMPUTER 15 /**< */
#define BEACON_ADDR_ANONYMOUS 16 /**< Id for beacons that will NEVER send messages (but only act as TDoA tags) */

/*
	This the data that is sent over the transmitter (we do NOT use the MAC frameing)
	- When first turned on, each beacon will connect to the WiFi network and the main computer will send them a 128bit AES key to use
	- Every single outbound packet will be encrypted using this key
	- In order for an inbound packet to be valid:
	 	- It is first decrypted using one of the up to 2 last stored keys
		- The 'type' must be a valid type
		- The 'mirror' bytes must match the first part of the message
		- The 'tx_time' must have a reasonable time difference compared to the last packet received
	- Every 1 minute, the computer will send a new key to every beacon via WiFi
		- Once every key is acknowledged, a second message is sent to every beacon to let them know that they can start using that key
*/
typedef struct {
	uint8_t type;
	uint8_t seq;

	// 0 is the broadcast address. Up to 15 devices are allowed to be uniquely target
	// This is just fine because going over 8 anchors is complicated for other reasons anyway
	uint8_t src_addr : 4;
	uint8_t dst_addr : 4;

	uint8_t data[];

} __attribute__((packed)) BeaconPacket;


typedef struct {
	uint8_t real_dst_addr;
} __attribute__((packed)) BeaconPacketProxyPing;

typedef struct {
	uint8_t delta_time[5];
} __attribute__((packed)) BeaconPacketProxyPingAck;

typedef struct {
	uint8_t time[5]; /**< Time on the anchor's clock when this broadcast was sent */
} __attribute__((packed)) BeaconPacketBroadcast;

typedef struct {
	uint8_t phase; /**<  */
	uint8_t total; /**< The total number of anchors in the system */
} __attribute__((packed)) BeaconPacketAnchorConfig;

typedef struct {
	// These two are sampled from the DW1000 directly
	uint8_t voltage_reg;
	uint8_t temperature;

	// Measured by the adapter board
	uint8_t voltage_battery;

	// TODO: Implement these
	uint8_t signal_strength;
	uint8_t clock_skew; // A rough quality metric on how skewed the beacons time appears to be compared to ones around it
} __attribute__((packed)) BeaconPacketStatAck;
