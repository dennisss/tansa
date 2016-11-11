#include <tansa/core.h>

extern void time_init();
extern void messaging_init();

namespace tansa
{


void init() {
	time_init();
	messaging_init();
}



}
