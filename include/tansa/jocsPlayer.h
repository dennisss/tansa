#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include "tansa/jocsParser.h"
#include "tansa/config.h"
class Mocap;
class GazeboConnector;

namespace tansa {
	class JocsPlayer {
	public:
		JocsPlayer(std::string jocsPath, double scale);
		Time play(vector<Vehicle *> vehicles, Time start, int i, int n, int &numLanded, bool &running, std::vector<unsigned> jocsActiveIds);
		void pause();
		void rewind(int steps);
		void reset();
		void loadJocs(std::string jocsPath, double scale);
		std::vector<Point> getHomes();
		std::vector<std::vector<Action*>> getActions();
		void initControllers(int n, std::vector<Vehicle *> vehicles, std::vector<unsigned> jocsActiveIds);
		void cleanup();
	private:
		Jocs* currentJocs;
		std::vector<std::vector<Action*>> actions;
		std::vector<Point> homes;
		std::vector<HoverController *> hovers;
		std::vector<PositionController *> posctls;
		std::vector<Trajectory *> takeoffs;
		std::vector<int> states;
		std::vector<int> plans;
		bool pauseRequested = false;
		bool resetMode = false;
		bool initialized = false;
	};
}
#endif //TANSA_JOCSPLAYER_H
