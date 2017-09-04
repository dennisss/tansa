#include <tansa/core/context.h>


namespace tansa {



Context::Context() : status(1) {


}

Context::~Context() {


}


void Context::poll() {
	unsigned out;
	if(!status.pop(&out, false))
		return;

	// Loop through all subscribers and check which onces have messages
	for(auto it = subs.begin(); it != subs.end(); ++it) {
		auto arr = it->second;
		for(auto it2 = arr.begin(); it2 != arr.end(); ++it2) {
			Subscription *s = *it2;
			s->call();
		}
	}
}



}
