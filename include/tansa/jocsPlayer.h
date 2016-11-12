#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include "tansa/jocsParser.h"

namespace tansa {
	class JocsPlayer {
	public:
		JocsPlayer(Jocs jocsData) : choreography(jocsData) {}
		static void play();
		static void pause();
		static void rewind(int steps);
		static void reset();
	private:
		Jocs choreography;
		bool running = false;
	};
}
#endif //TANSA_JOCSPLAYER_H
