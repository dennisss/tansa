#include <tansa/core.h>

extern void time_init();
extern void sim_init();

namespace tansa
{


void init() {
	time_init();
	sim_init();
}



}
