#include "../vehicle.h"
#include "connection_pool.h"

namespace tansa {
namespace cf {

class Vehicle : tansa::Vehicle {





private:

	void handle_message(crtp_message *msg);


	ConnectionPool *pool;
	RadioUrl uri;
	int connected;
	int success_count, fail_count;
}


}
}