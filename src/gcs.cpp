#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/jocsParser.h>
#include <tansa/config.h>
#include <tansa/jocsPlayer.h>
#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <tansa/osc.h>

#ifdef  __linux__
#include <sys/signal.h>
#endif

#include <iostream>

using namespace std;
using namespace tansa;

static bool running;
static JocsPlayer* player;
static vector<Vehicle *> vehicles;

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

void socket_on_message(sio::message::ptr const& data) {
	string type = data->get_map()["type"]->get_string();

	if(type == "prepare") {
		printf("Preparing...\n");
		player->prepare();
	}
	else if(type == "play") {
		printf("Playing...\n");
		player->play();
	} else if (type == "pause") {
		printf("Pausing...\n");
	} else if (type == "reset") {
		printf("Resetting...\n");
	}
	else {
		// TODO: Send an error message back to the browser
		printf("Unexpected message type recieved!\n");
	}

}


void osc_on_message(OSCMessage &msg) {

	// Address will look something like: '/cue/0101/start'
	if(msg.address[0] == "cue") {

		int num = std::stoi(msg.address[1]);

		if(msg.address[2] == "load") {
			player->prepare();
		}
		else if(msg.address[2] == "start") {
			printf("Starting at cue #: %d\n", num);

			// Assert that it is already prepared at the given cue

			player->play();
		}

	}

	/*
	printf("Address:\n");
	for(auto str : msg.address)
		printf("- %s\n", str.c_str());

	printf("\nArgs:\n");
	for(auto str : msg.args)
		printf("- %s\n", str.c_str());
	*/

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



		if(args[0] == "prepare") {
			cout << "Preparing..." << endl;
			player->prepare();
		}
		else if(args[0] == "play") {
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
	bool enableOSC = rawJson["enableOSC"];

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

	Jocs *jocs = Jocs::Parse(jocsPath, scale);

	auto homes = jocs->GetHomes();

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

	vehicles.resize(n);

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

	if(enableOSC) {
		OSC *osc = new OSC();
		osc->start(53100);
		osc->set_listener(osc_on_message);
	}




	player = new JocsPlayer(vehicles, jocsActiveIds);
	player->loadJocs(jocs);

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

		player->step();

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
