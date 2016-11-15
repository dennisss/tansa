#include <tansa/core.h>

extern void time_init();
extern void messaging_init();

namespace tansa
{


void init(bool enableMessaging) {
	time_init();
	if(enableMessaging) {
		messaging_init();
	}
}



}
