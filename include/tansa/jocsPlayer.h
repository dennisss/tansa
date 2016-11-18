#ifndef TANSA_JOCSPLAYER_H
#define TANSA_JOCSPLAYER_H

#include "tansa/jocsParser.h"
#include "tansa/config.h"

namespace tansa {
	class JocsPlayer {
	public:
		static const unsigned STATE_INIT;
		static const unsigned STATE_TAKEOFF;
		static const unsigned STATE_FLYING;
		static const unsigned STATE_LANDING;
		JocsPlayer(bool withMocap, hardware_config config, string jocsPath, std::vector<unsigned> jocsActiveIds, double scale);
		Time play(Time start, int i, int &numLanded, bool &running, std::vector<unsigned> jocsActiveIds);
		void pause();
		void rewind(int steps);
		void reset();
		void loadJocs(string jocsPath, std::vector<unsigned> jocsActiveIds, double scale);
		void initVehicles(std::vector<vehicle_config> vconfigs, std::vector<unsigned> jocsActiveIds);
		void cleanup();
		bool isInitialized();
	private:
		std::vector<Point> spawns;
		std::vector<Point> homes;
		std::vector<std::vector<Action*>> actions;
		std::vector<Vehicle *> vehicles;
		std::vector<HoverController *> hovers;
		std::vector<PositionController *> posctls;
		std::vector<Trajectory *> takeoffs;
		std::vector<int> states;
		std::vector<int> plans;
		Mocap *mocap = nullptr;
		GazeboConnector *gazebo = nullptr;
		bool useMocap;
		bool pauseRequested = false;
		bool resetMode = false;
		bool initialized = false;
		int n;
	};
}
#endif //TANSA_JOCSPLAYER_H
