#ifndef TANSA_MOCAP_H_
#define TANSA_MOCAP_H_

#include "vehicle.h"

#include <map>

extern class NatNetClient;

/**
 * Used to interface pose feedback from motion capture systems with the vehicles
 */
class Mocap {
public:
	Mocap();

	/**
	 * Connect to the motion capture software
	 *
	 * @param iface_addr local ip address of the network interface through which the data is being streamed
	 */
	int connect(string iface_addr);

	
	int disconnect();

	/**
	 *
	 */
	void track(Vehicle *v, int id);

private:

	map<int, Vehicle *> tracked;
	NatNetClient* client;

};


#endif
