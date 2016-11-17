#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include "tansa/jocsParser.h"

namespace tansa {
	class JocsPlayer {
	public:
		JocsPlayer(bool useMocap) : useMocap(useMocap)  {}
		void play();
		void pause();
		void rewind(int steps);
		void reset();
		void loadJocs(string jocsPath);
		void loadConfig(string configPath);
	private:
		Jocs jocsData;
		std::vector<Point> homes;
		std::vector<std::vector<Action*>> actions;
		bool useMocap;
		bool running = false;
		bool pauseRequested = false;
		bool resetMode = false;
	};
}
#endif //TANSA_JOCSPLAYER_H
