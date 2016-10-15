#ifndef TANSA_MOCAP_H_
#define TANSA_MOCAP_H_

#include "vehicle.h"

#include <map>

class NatNetClient;
struct sFrameOfMocapData;
void mocap_callback(struct sFrameOfMocapData* pFrameOfData, void* pUserData);

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
	int connect(string iface_addr, string server_addr);


	int disconnect();

	/**
	 * Explictly specifies that a Vehicle is being tracked by a specific rigid body in the mocap system
	 *
	 * @param v the vehicle being tracked
	 * @param id the id number of the body in Motive
	 */
	void track(Vehicle *v, int id);

private:

	friend void mocap_callback(struct sFrameOfMocapData * pFrameOfData, void* pUserData);

	map<int, Vehicle *> tracked;
	NatNetClient* client;

};


#endif
