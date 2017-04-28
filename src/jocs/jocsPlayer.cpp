#include "tansa/jocsPlayer.h"
#include <tansa/algorithm.h>

#include <unistd.h>

namespace tansa {


void JocsPlayer::play() {

	// TODO: We should also check that the drones are in the right place (a Stop may get them to the wrong position)

	for(auto s : states) {
		if(s != StateHolding) {
			printf("Cannot play: Some drones not ready\n");
			return;
		}
	}

	if (paused) {
		paused = false;
		pauseRequested = false;
		stopRequested = false;
		timeOffset += Time::now().since(pauseOffset).seconds();
		int n = jocsActiveIds.size();
		for (int i = 0; i < n; i++) {
			plans[i] = pauseIndices[i];
		}
	} else {
		start = Time::now();
		reset();
	}

	for(auto &s : states) {
		s = StateFlying;
	}
}

	/**
	 * Pause the choreography
	 */
	void JocsPlayer::pause() {
		pauseRequested = true;
		// TODO: Determine a pause-at index (and maybe also a stop-at index)
	}

	double JocsPlayer::currentTime() {
		if(!this->isPlaying()) {
			return -1;
		}

		return Time::now().since(start).seconds() + startOffset;
	}


}
