#include <tansa/core.h>

namespace tansa {

extern void time_init();
extern void messaging_init();
extern void messaging_end();

bool didEnableMessaging;

void init(bool enableMessaging) {
	time_init();
	didEnableMessaging = enableMessaging;
	if(enableMessaging) {
		messaging_init();
	}
}

void end() {
	if(didEnableMessaging) {
		messaging_end();
	}
}


}
