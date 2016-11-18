#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/jocsParser.h>
#include <tansa/config.h>
#include <tansa/jocsPlayer.h>
#include <tansa/mocap.h>
#include <tansa/gazebo.h>

#ifdef  __linux__
#include <sys/signal.h>
#endif

#include <iostream>

using namespace std;
using namespace tansa;

static bool running;
static JocsPlayer* player;


void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}

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
	} else if (type == "pause") {
		printf("Pausing...\n");
	} else if (type == "reset") {
		printf("Resetting...\n");
	} else {
		throw "Unexpected message type recieved!";
	}

}

pthread_t console_handle;

void *console_thread(void *arg) {

	while(running) {

		// Read a command
		cout << "> ";
		string line;
		getline(cin, line);

		// Split into arguments
		vector<string> args;
		istringstream iss(line);
		while(!iss.eof()) {
			string a;
			iss >> a;

			if(iss.fail())
				break;

			args.push_back(a);
		}


		if(args.size() == 0)
			continue;



		if(args[0] == "play") {
			cout << "Playing..." << endl;
			player->play();
		}


	}

}

void console_start() {
	pthread_create(&console_handle, NULL, console_thread, NULL);
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

	player = new JocsPlayer(jocsPath, scale);
	auto homes = player->getHomes();

	// Only pay attention to homes of active drones
	std::vector<Point> spawns;
	for (int i = 0; i < jocsActiveIds.size(); i++) {
		int chosenId = jocsActiveIds[i];
		// We assume the user only configured for valid IDs..
		spawns.push_back(homes[chosenId]);
		spawns[i].z() = 0;
	}

	tansa::init(enableMessaging);

	Mocap *mocap = nullptr;
	GazeboConnector *gazebo = nullptr;

	// Only pay attention to homes of active drones
	// TODO: Have a better check for mocap initialization/health
	if (useMocap) {
		mocap = new Mocap();
		mocap->connect(config.clientAddress, config.serverAddress);
	} else {
		gazebo = new GazeboConnector();
		gazebo->connect();
		gazebo->spawn(spawns);
	}

	int n = spawns.size();

	if (n > vconfigs.size()) {
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

	player->initControllers(vehicles, jocsActiveIds);

	running = true;
	signal(SIGINT, signal_sigint);

	int i = 0;

	/*
	// For sample lighting demo
	float level = 0;
	float dl = 0.005;
	*/

	signal(SIGINT, signal_sigint);
	printf("running...\n");
	running = true;

	console_start();

	Rate r(100);
	while(running) {

		// Regular status messages
		if(enableMessaging && i % 20 == 0) {
			send_status_message();
		}

		player->step(vehicles, jocsActiveIds);

		r.sleep();
		i++;
	}

	/// Cleanup
	if (useMocap) {
		mocap->disconnect();
		delete mocap;
	} else {
		gazebo->disconnect();
		delete gazebo;
	}

	// Stop all vehicles
	for(int vi = 0; vi < n; vi++) {
		Vehicle *v = vehicles[vi];
		v->disconnect();
		delete v;
	}

	player->cleanup();

	printf("Done!\n");
}
