#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/gazebo.h>
#include <tansa/jocsParser.h>
#include <tansa/mocap.h>

#include <unistd.h>
#include <sys/signal.h>

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

struct vehicle_config {
	unsigned net_id; // The number printed on the physical
	unsigned chor_id; // The id between 1 and 6 respesenting which drone in the choreography it w
	unsigned lport; // Usually 14550 + id*10
	unsigned rport; // For now always 14555
};

/* For sending a system state update to the gui */
void send_status_message() {
	Time t = Time::now();

	sio::message::ptr obj = sio::object_message::create();
	obj->get_map()["type"] = sio::string_message::create("status");
	obj->get_map()["time"] = sio::double_message::create(t.seconds());

	sio::message::list li(obj);
	tansa::send_message(li);

}

void on_message(sio::message::ptr const& data) {
	string type = data->get_map()["type"]->get_string();

	if(type == "play") {
		printf("Playing...\n");
	}

}


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
	vector<unsigned> jocsActiveIds = rawJson["jocsActiveIds"];
	bool useMocap = rawJson["useMocap"];
	float scale = rawJson["theaterScale"];
	bool enableMessaging = rawJson["enableMessaging"];

	if (useMocap) {
		nlohmann::json hardwareConfig = rawJson["hardwareConfig"];
		config.clientAddress = hardwareConfig["clientAddress"];
		config.serverAddress = hardwareConfig["serverAddress"];
	}

	std::vector<vehicle_config> vconfigs(rawJson["vehicles"].size());
	for(unsigned i = 0; i < rawJson["vehicles"].size(); i++) {
		vconfigs[i].net_id = rawJson["vehicles"][i]["net_id"];
		if(useMocap) {
			vconfigs[i].lport = 14550 + 10*vconfigs[i].net_id;
			vconfigs[i].rport = 14555;
		}
		else { // The simulated ones are zero-indexed and
			vconfigs[i].lport = 14550 + 10*(vconfigs[i].net_id - 1);
			vconfigs[i].rport = 14555 + 10*(vconfigs[i].net_id - 1);
		}
	}


	tansa::init(enableMessaging);
	auto jocsData = Jocs::Parse(jocsPath, scale);
	auto actions = jocsData.GetActions();
	auto homes = jocsData.GetHomes();

	// Only pay attention to homes of active drones
	std::vector<Point> spawns;
	for (int i = 0; i < jocsActiveIds.size(); i++) {
		int chosenId = jocsActiveIds[i];
		// We assume the user only configured for valid IDs..
		spawns.push_back(homes[chosenId]);
        spawns[i].z() = 0;
	}


	Mocap *mocap = nullptr;
	GazeboConnector *gazebo = nullptr;

	if (useMocap) {
		mocap = new Mocap();
		mocap->connect(config.clientAddress, config.serverAddress);
	} else {
		gazebo = new GazeboConnector();
		gazebo->connect();
		gazebo->spawn(spawns);
	}


	int n = spawns.size();

	if(n > vconfigs.size()) {
		printf("Not enough drones on the network\n");
		return 1;
	}

	vector<Vehicle *> vehicles(n);
	for(int i = 0; i < n; i++) {
		const vehicle_config &v = vconfigs[i];

		vehicles[i] = new Vehicle();
		vehicles[i]->connect(v.lport, v.rport);
		if (useMocap) {
			mocap->track(vehicles[i], i+1);
		} else {
			gazebo->track(vehicles[i], i);
		}
	}

	// TODO: Have a better check for mocap initialization/health
	sleep(15);

	vector<HoverController *> hovers(n);
	for(int i = 0; i < n; i++) {
		hovers[i] = new HoverController(vehicles[i], homes[jocsActiveIds[i]]);
	}


	vector<PositionController *> posctls(n);
	for(int i = 0; i < n; i++) {
		posctls[i] = new PositionController(vehicles[i]);
	}


	vector<Trajectory *> takeoffs(n);
	for(int i = 0; i < n; i++) {
		takeoffs[i] = new LinearTrajectory(vehicles[i]->state.position, 0, homes[jocsActiveIds[i]], 10.0);
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

	// Index of the current action running for each drone (initially running the 0th)
	std::vector<int> plans(n, 0);

	printf("running...\n");

	Rate r(100);
	while(running) {

		//r.sleep();
		//continue;

		double t = Time::now().since(start).seconds();


		// Regular status messages
		if(enableMessaging && i % 20 == 0) {
			send_status_message();
		}

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

				if(t >= actions[jocsActiveIds[vi]][plans[vi]]->GetEndTime()) {
					if(plans[vi] == actions[jocsActiveIds[vi]].size()-1) {
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
				Trajectory *cur = static_cast<MotionAction*>(actions[jocsActiveIds[vi]][plans[vi]])->GetPath();
				posctls[vi]->track(cur);
				posctls[vi]->control(t);
			}
		}
		r.sleep();
		i++;
	}

	/// Cleanup
	if (useMocap) {
		mocap->disconnect();
		delete mocap;
	}
	else {
		gazebo->disconnect();
		delete gazebo;
	}

	// Stop all vehicles
	for(int vi = 0; vi < n; vi++) {
		Vehicle *v = vehicles[vi];
		v->disconnect();
		delete v;
	}

	for(int i = 0; i < posctls.size(); i++){
		delete posctls[i];
	}

	for(int i = 0; i < hovers.size(); i++){
		delete hovers[i];
	}

	for(int i = 0; i < takeoffs.size(); i++){
		delete takeoffs[i];
	}

	printf("Done!\n");
}
