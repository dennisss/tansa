#ifndef TANSA_CONFIG_H_
#define TANSA_CONFIG_H_

/**
 * Contains client and server address for drone netcode.
 */
struct hardware_config {
	string clientAddress;
	string serverAddress;
};

/**
 * Contains config info for drone
 */
struct vehicle_config {
	unsigned net_id; // The number printed on the physical
	unsigned chor_id; // The id between 1 and 6 respesenting which drone in the choreography it w
	unsigned lport; // Usually 14550 + id*10
	unsigned rport; // For now always 14555
};


#endif //TANSA_CONFIG_H
