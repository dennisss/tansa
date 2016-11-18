#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/jocsParser.h>
#include <tansa/config.h>
#include <tansa/jocsPlayer.h>

using namespace tansa;
bool running;

void signal_sigint(int s) {
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

	auto player = new JocsPlayer(useMocap, config, jocsPath, jocsActiveIds, scale);

	player->initVehicles(vconfigs, jocsActiveIds);
	assert(player->isInitialized());

	int numLanded = 0;

	int i = 0;

	/*
	// For sample lighting demo
	float level = 0;
	float dl = 0.005;
	*/

	Time start(0,0);

	signal(SIGINT, signal_sigint);
	printf("running...\n");

	Rate r(100);
	while(running) {
		// Regular status messages
		if(enableMessaging && i % 20 == 0) {
			send_status_message();
		}

		start = player->play(start, i, numLanded, running, jocsActiveIds);
		r.sleep();
		i++;
	}

	/// Cleanup
	player->cleanup();

	printf("Done!\n");
}
