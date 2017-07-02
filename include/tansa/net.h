#ifndef TANSA_NET_H_
#define TANSA_NET_H_

#include <netinet/ip.h>

#include <string>
#include <vector>

/*
	Abstraction layer over OS specific network sockets etc.
*/

namespace tansa {

class NetworkAddress {
public:

	NetworkAddress();
	NetworkAddress(const struct sockaddr *raw);
	NetworkAddress(const char *ip, int port = 0);

	/**
	 * Apply a netmask to the address. This will zero the port and all parts not in the mask
	 */
	NetworkAddress masked(const NetworkAddress &m) const;

	bool isLoopback() const;

	bool operator==(const NetworkAddress &other) const;

	const struct sockaddr_in *raw() { return &addr; }

private:
	struct sockaddr_in addr;
};


class NetworkInterface {
public:

	/**
	 * Get a list of all interfaces in the system
	 */
	static std::vector<NetworkInterface> all();

	/**
	 * Given some ip address on an interface's subnet, finds that interface. Returns wether or not it could be found
	 */
	static bool find(const NetworkAddress &addr, NetworkInterface *out);

	std::string name;
	NetworkAddress address;
	NetworkAddress broadcast;
	NetworkAddress mask;

private:

};


class Socket {
public:

	enum Flags {
		NonBlocking = 1,
		Broadcast = 2,
		Reuseable = 4
	};

	enum Type {
		UDP,
		TCP
	};

	bool open(Type t, int flags = 0);

	bool bind(const NetworkAddress &addr);


	int fd() { return this->sock; }

private:
	int sock = -1;
};


}




#endif
