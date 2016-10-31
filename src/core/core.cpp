#include <tansa/core.h>

namespace tansa {

extern void time_init();
extern void messaging_init();

void init(bool enableMessaging) {
	time_init();
	if(enableMessaging) {
		messaging_init();
	}
}



}
