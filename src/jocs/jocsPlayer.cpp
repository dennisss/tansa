#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <zconf.h>
#include <tansa/control.h>
#include "tansa/jocsPlayer.h"
#include <unistd.h>
#include <sys/signal.h>

namespace tansa {

	/**
	 * Begin to play the choreography
	 */
	void JocsPlayer::play() {

	}

	/**
	 * Pause the choreography
	 */
	void JocsPlayer::pause() {
		JocsPlayer::pauseRequested = true;
	}

	/**
	 * Rewind the chreography by a number of 'steps'
	 * @param steps How far back in the choreography to rewind
	 */
	void JocsPlayer::rewind(int steps) {

	}

	/**
	 * Reset the choreography back to the initial position (ie: plans[0])
	 */
	void JocsPlayer::reset() {
		JocsPlayer::resetMode = true;
	}

    /**
     * Load JOCS data from a specified path
     */
    void JocsPlayer::loadJocs(string jocsPath) {
		// TODO: Clear all these if they already have data.
		JocsPlayer::jocsData = Jocs::Parse(jocsPath);
		JocsPlayer::homes = jocsData.GetHomes();
		JocsPlayer::actions = jocsData.GetActions();
    }
}
