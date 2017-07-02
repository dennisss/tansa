#include <tansa/net.h>

#include <string.h>
#include <ifaddrs.h>
#include <arpa/inet.h>


namespace tansa {



NetworkAddress::NetworkAddress() {
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
}

NetworkAddress::NetworkAddress(const struct sockaddr *raw) {
	memcpy(&this->addr, raw, sizeof(struct sockaddr_in));
}


NetworkAddress::NetworkAddress(const char *ip, int port) : NetworkAddress() {
	addr.sin_port = htons(port);
	inet_pton(AF_INET, ip, &addr.sin_addr); // TODO: Error check this
}


NetworkAddress NetworkAddress::masked(const NetworkAddress &m) const {
	NetworkAddress masked;
	masked.addr.sin_addr.s_addr = this->addr.sin_addr.s_addr & m.addr.sin_addr.s_addr;
	return masked;
}

bool NetworkAddress::isLoopback() const {
	return addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK);
}

bool NetworkAddress::operator==(const NetworkAddress &other) const {
	return this->addr.sin_addr.s_addr == other.addr.sin_addr.s_addr
		&& this->addr.sin_port == other.addr.sin_port;
}


std::vector<NetworkInterface> NetworkInterface::all() {

	std::vector<NetworkInterface> arr;

	struct ifaddrs *ifap, *ifa;
	getifaddrs(&ifap);
	for(ifa = ifap; ifa; ifa = ifa->ifa_next) {
		if(ifa->ifa_addr->sa_family == AF_INET) {
			NetworkInterface ni;
			ni.name = std::string(ifa->ifa_name);
			ni.address = NetworkAddress(ifa->ifa_addr);
			ni.broadcast = NetworkAddress(ifa->ifa_broadaddr);
			ni.mask = NetworkAddress(ifa->ifa_netmask);
			arr.push_back(ni);
		}
	}

	return arr;
}

bool NetworkInterface::find(const NetworkAddress &addr, NetworkInterface *out) {

	std::vector<NetworkInterface> ifaces = NetworkInterface::all();

	for(const NetworkInterface &iface : ifaces) {

		if(iface.address.isLoopback())
			continue;

		if(iface.address.masked(iface.mask) == addr.masked(iface.mask)) {
			*out = iface;
			return true;
		}
	}

	return false;
}





bool Socket::open(Type t, int flags) {

	this->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);


	int val;


#ifdef SO_REUSEPORT
	val = 1;
	if(setsockopt(this->sock, SOL_SOCKET, SO_REUSEPORT, &val, sizeof(val)) == -1){
		printf("Failed to make socket reusable\n");
		return false;
	}
#endif


	val = 1;
	if(setsockopt(this->sock, SOL_SOCKET, SO_BROADCAST, (void *)&val, sizeof(val)) == -1) {
		perror("setsockopt (SO_BROADCAST)");
		return false;
	}

	return true;
}


}
