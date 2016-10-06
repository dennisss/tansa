#ifndef TANSA_CORE_H_
#define TANSA_CORE_H_

// TODO: Eventually, don't have this here
#include "vehicle.h"

namespace tansa {


/**
 * Call this on program start before using the other functions of the library
 */
void init();


void sim_connect();
void sim_disconnect();
void sim_track(Vehicle *veh, int id);

}




#endif
