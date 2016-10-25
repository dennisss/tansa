#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/gazebo.h>
#include <tansa/jocsParser.h>
#include <tansa/mocap.h>

#include <unistd.h>

using namespace tansa;
bool running;

void signal_sigint(int s) {
	running = false;
}

#define STATE_INIT 0
#define STATE_TAKEOFF 1
#define STATE_FLYING 2
#define STATE_LANDING 3

struct hardware_config {
	string clientAddress;
	string serverAddress;
};

int main(int argc, char *argv[]) {

	assert(argc == 2);
	string configPath = argv[1];

	ifstream configStream(configPath);
	if (!configStream) throw "Unable to read config file!";

	/// Parse the config file
	std::string configData((std::istreambuf_iterator<char>(configStream)), std::istreambuf_iterator<char>());
	nlohmann::json rawJson = nlohmann::json::parse(configData);
	hardware_config config;
	string jocsPath = rawJson["jocsPath"];
	bool useMocap = rawJson["useMocap"];

	if (useMocap) {
		nlohmann::json hardwareConfig = rawJson["hardwareConfig"];
		config.clientAddress = hardwareConfig["clientAddress"];
		config.serverAddress = hardwareConfig["serverAddress"];
	}

	tansa::init();
	auto data = Jocs(jocsPath);
	auto actions = data.Parse();
	auto homes = data.GetHomes();
	std::vector<Point> spawns = homes;
	for(auto& s : spawns){
		s.z() = 0;
	}


	Mocap *mocap;
	GazeboConnector *gazebo;

	if (useMocap) {
		mocap = new Mocap();
		mocap->connect(config.clientAddress, config.serverAddress);
	} else {
		gazebo = new GazeboConnector();
		gazebo->connect();
		gazebo->spawn(spawns);
	}


	int n = homes.size();


	vector<Vehicle *> vehicles(n);
	for(int i = 0; i < n; i++) {
		vehicles[i] = new Vehicle();
		vehicles[i]->connect(14550 + i*10, 14555 + i*10);
		if (useMocap) {
			mocap->track(vehicles[i], i+1);
		} else {
			gazebo->track(vehicles[i], i);
		}
	}

	// TODO: Have a better check for mocap initialization/health
	sleep(10);

	vector<HoverController *> hovers(n);
	for(int i = 0; i < n; i++) {
		hovers[i] = new HoverController(vehicles[i], homes[i]);
	}


	vector<PositionController *> posctls(n);
	for(int i = 0; i < n; i++) {
		posctls[i] = new PositionController(vehicles[i]);
	}


	vector<Trajectory *> takeoffs(n);
	for(int i = 0; i < n; i++) {
		takeoffs[i] = new LinearTrajectory(vehicles[i]->state.position, 0, homes[i], 10.0);
	}

	int numLanded = 0;
	std::vector<int> states;
	states.resize(n);
	for(auto& state : states){
		state = STATE_INIT;
	}
	running = true;
	signal(SIGINT, signal_sigint);

	int i = 0;

	/*
	// For sample lighting demo
	float level = 0;
	float dl = 0.005;
	*/

	Time start(0,0);

	std::vector<int> plans;
	plans.resize(n);
	//This might zero memory already have to check if this is necessary
	for(auto& p : plans){
		p = 0;
	}

	printf("running...\n");

	Rate r(100);
	while(running) {

		//r.sleep();
		//continue;

		double t = Time::now().since(start).seconds();


		// Check for state transitions
		if(states[0] == STATE_INIT) {

			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if(vehicles[vi]->mode != "offboard" || !vehicles[vi]->armed) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				for(auto& state :states) {
					state = STATE_TAKEOFF;
				}
				continue;
			}

		}
		else if(states[0] == STATE_TAKEOFF) {
			if(t >= 10.0) {
				for(auto& state : states){
					state = STATE_FLYING;
				}
				start = Time::now();
				continue;
			}

			/*
			bool allGood = true;
			for(int vi = 0; vi < n; vi++) {
				if(hovers[vi]->distance() > 0.1) {
					allGood = false;
					break;
				}
			}

			if(allGood) {
				start = Time::now();
				state = STATE_FLYING;
			}
			*/
		}


		// Do the control loops
		for(int vi = 0; vi < n; vi++) {
			Vehicle &v = *vehicles[vi];

			/*
			Sample Lighting stuff

			v.set_lighting(level, level);

			level += dl;
			if(level >= 1.0 || level <= 0.0)
				dl = -dl;
			*/

			if(states[vi] == STATE_INIT) {
				// Lower frequency state management
				if(i % 50 == 0) {

					if(v.mode != "offboard") {
						v.set_mode("offboard");
						printf("Setting mode\n");
					}
					else if(!v.armed) {
						v.arm(true);
						printf("Arming mode\n");
					}
				}


				// Do nothing
				vehicles[vi]->setpoint_accel(Vector3d(0,0,0));
			}
			else if(states[vi] == STATE_TAKEOFF) {
				posctls[vi]->track(takeoffs[vi]);
				posctls[vi]->control(t);
			}
			else if(states[vi] == STATE_FLYING) {

				if(t >= actions[vi][plans[vi]]->GetEndTime()) {
					if(plans[vi] == actions[vi].size()-1) {
						states[vi] = STATE_LANDING;
						v.land();
						numLanded++;
						if(numLanded == n){
							running = false;
						}
						continue;
					}
					plans[vi]++;
				}
				Trajectory *cur = static_cast<MotionAction*>(actions[vi][plans[vi]])->GetPath();
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}
		}
		r.sleep();
		i++;
	}

	/// Cleanup

	if (!useMocap) {
		gazebo->disconnect();
	}


	// Stop all vehicles
	for(int vi = 0; vi < n; vi++) {
		Vehicle &v = *vehicles[vi];
		v.disconnect();
	}

	printf("Done!\n");
}
