#ifndef TANSA_MANAGER_SAFETY_CHECKS_H_
#define TANSA_MANAGER_SAFETY_CHECKS_H_

#include <tansa/vehicle.h>

/**
 * Watches a single vehicle for any issues
 */
class SafetyChecksHelper {

public:

	SafetyChecksHelper(Vehicle::ptr v) { this->vehicle = vehicle; }


	bool check();


private:

	Vehicle::Ptr vehicle;

};


#endif
