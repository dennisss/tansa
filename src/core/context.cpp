#include <tansa/core/context.h>


namespace tansa {



Context::Context() : status(0) {


}


void Context::poll() {

	while(status->pop()) {

		// Loop through all subscribers and check which onces have messages

	}

}



}
