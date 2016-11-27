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
//TODO check if these work on OSX
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace tansa;

// TODO: Resolve this to an absolute path
static const char *paramsDir = "./config/params/";

static bool running = false;
static bool initialized = false;
static bool killmode = false;
static bool pauseMode = false;
static bool stopMode = false;
static bool playMode = false;
static bool prepareMode = false;
static bool loadMode = false;
static float scale = 1.0;
static JocsPlayer* player = nullptr;
static GazeboConnector *gazebo = nullptr;
static Mocap *mocap = nullptr;
static vector<Vehicle *> vehicles;
static std::vector<vehicle_config> vconfigs;
static vector<unsigned> jocsActiveIds;
static bool useMocap;
static string jocsConfigPath = "src/jocsConfig.json";

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}

/**
 * Construct an array of error objects to include with a response when an error has occurred.
 * @param message	The message to be displayed by the GUI
 */
void generateError(json &response, string message) {
	json error;
	error["message"] = message;
	if (response.find("error") != response.end()) {
		response["error"].push_back(error);
	} else {
		json errorResponse = json::array();
		errorResponse.push_back(error);
		response["error"] = errorResponse;
	}
}

/* For sending a system state update to the gui */
void send_status_message() {
	json jsonStatus;

	jsonStatus["type"] = "status";
	jsonStatus["time"] = player->currentTime();

	json jsonVehicles = json::array();
	for(int i = 0; i < vehicles.size(); i++) {
		json jsonVehicle;
		jsonVehicle["id"] = vconfigs[i].net_id;
		jsonVehicle["role"] = (jocsActiveIds.size() > 0 && i <= jocsActiveIds.size() - 1) ? jocsActiveIds[i] : -1;
		jsonVehicle["connected"] = vehicles[i]->connected;
		jsonVehicle["armed"] = vehicles[i]->armed;
		jsonVehicle["tracking"] = vehicles[i]->tracking;

		json jsonPosition = json::array();
		jsonPosition.push_back(vehicles[i]->state.position.x());
		jsonPosition.push_back(vehicles[i]->state.position.y());
		jsonPosition.push_back(vehicles[i]->state.position.z());
		jsonVehicle["position"] = jsonPosition;

		json jsonBatteryStats = {
			{"voltage", vehicles[i]->battery.voltage},
			{"percent", vehicles[i]->battery.percent}
		};
		jsonVehicle["battery"] = jsonBatteryStats;

		jsonVehicles.push_back(jsonVehicle);
	}
	jsonStatus["vehicles"] = jsonVehicles;

	json globalStatus;
	globalStatus["playing"] = player->isPlaying();
	globalStatus["ready"] = player->isReady();
	globalStatus["paused"] = player->isPaused();
	globalStatus["landed"] = player->isLanded();
	jsonStatus["global"] = globalStatus;

	tansa::send_message(jsonStatus);
}

void send_file_list() {
	json j;

	j["type"] = "list_reply";
	json files = json::array();
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir ("data")) != NULL) {
		while ((ent = readdir (dir)) != NULL) {
			if(ent->d_type == DT_REG && std::string(ent->d_name).find(".jocs") != std::string::npos)
				files.push_back(std::string(ent->d_name));
		}
		closedir (dir);
	} else {
		generateError(j, "Could not open directory.");
	}
	j["files"] = files;
	tansa::send_message(j);
}

/**
 * Disconnect and delete any existing vehicles, and initialize those specified via JSON
 * @param rawJson 			The JSON object containing the vehicle configuration information (net_id's)
 * @param homes 			Homes for the new vehicles, specified by the current Jocs file
 * @param jocsActiveIds 	The ids of the drones to be used in this configuration
 */
void spawnVehicles(const json &rawJson, vector<Point> homes, vector<unsigned> jocsActiveIds) {
	vconfigs.resize(rawJson["vehicles"].size());

	for (unsigned i = 0; i < rawJson["vehicles"].size(); i++) {
		vconfigs[i].net_id = rawJson["vehicles"][i]["net_id"];
		if (useMocap) {
			vconfigs[i].lport = 14550 + 10*vconfigs[i].net_id;
			vconfigs[i].rport = 14555;
		} else { // The simulated ones are zero-indexed and are always in ascending order
			vconfigs[i].net_id = i;

			vconfigs[i].lport = 14550 + 10*vconfigs[i].net_id;
			vconfigs[i].rport = 14555 + 10*vconfigs[i].net_id;
		}
	}

	// Number of drones used (limited by active ids and number of available slots in choreography)
	int n = jocsActiveIds.size();
	if(homes.size() < n) {
		n = homes.size();
		jocsActiveIds.resize(n);
	}

	if (n > vconfigs.size()) {
		printf("Not enough drones on the network\n");
		return;
	}

	// Stop all vehicles
	for (int vi = 0; vi < vehicles.size(); vi++) {
		Vehicle *v = vehicles[vi];
		v->disconnect();
		delete v;
	}

	vehicles.resize(n);

	for (int i = 0; i < n; i++) {
		const vehicle_config &v = vconfigs[i];

		vehicles[i] = new Vehicle();

		// Load default parameters
		vehicles[i]->readParams(string(paramsDir) + "default.json");
		string calibId = useMocap? to_string(v.net_id) : "sim";

		if(!vehicles[i]->readParams(string(paramsDir) + to_string(v.net_id) + ".calib.json")) {
			cout << "#" + to_string(v.net_id) + " not calibrated!" << endl;
		}

		vehicles[i]->connect(v.lport, v.rport);
		if (useMocap) {
			mocap->track(vehicles[i], i+1);
		} else {
			gazebo->track(vehicles[i], i);
		}
	}

	if (!useMocap) {
		// Only pay attention to homes of active drones
		vector<Point> spawns;
		for (int i = 0; i < n; i++) {
			int chosenId = jocsActiveIds[i];
			// We assume the user only configured for valid IDs..
			spawns.push_back(homes[chosenId]);
			spawns[i].z() = 0;
		}
		gazebo->spawn(spawns);
	}

	// Initialize the vehicles within the jocsPlayer
	player->initVehicles(vehicles);
}

/**
 * Create and send the JSON response after receieving the 'load' command from the GUI
 */
void constructLoadResponse() {
	auto breakpoints = player->getBreakpoints();
	json j;
	j["type"] = "load_reply";
	json nums = json::array();
	for (auto& b : breakpoints){
		nums.push_back(b.GetNumber());
	}
	json positions = json::array();
	for (Point home : player->getHomes()){
		//TODO: need to fill in the starting positions for breakpoints. Not contained in breakpoints currently
		json position;
		position["x"] = home.x();
		position["y"] = home.y();
		positions.push_back(position);
	}
	j["cues"] = nums;
	j["target_positions"] = positions;
	tansa::send_message(j);
}

/**
 * Load a Jocs file with the configuration parameters specified via either the REPL or the GUI
 * @param rawJson 	JSON containing configuration information to initialize (or reconfigure) the JocsPlayer
 */
void loadJocsFile(const json &rawJson) {
	if (player == nullptr) {
		// TODO: Send some kinda error code here
		printf("Player is null...\n");
		loadMode = false;
		return;
	} else if (!player->canLoad()) {
		printf("Player not in a state to load...\n");
		loadMode = false;
		return;
	}

	initialized = false;
	string jocsPath = rawJson["jocsPath"];
	vector<unsigned> jocsActiveIds = rawJson["jocsActiveIds"];
	scale = rawJson["theaterScale"];

	player->cleanup();
	player->loadJocs(jocsPath, scale, jocsActiveIds);
	vector<Point> homes = player->getHomes();
	spawnVehicles(rawJson, homes, jocsActiveIds);

	// int cue = data["cue"];
	// TODO: Prepare the jocs file to start at this cue.
	// TODO: Need to make sure this gets deleted. Will have to delete inside the JocsPlayer class when we load a new file.
	// In other words, we transfer ownership of the jocs object to the player here.

	constructLoadResponse();

	initialized = true;
	loadMode = false;
	printf("Finished loadJocsFile\n");
}

/**
 * Legacy method to load entirely from a config file, allows for initialization/loading via the REPL
 * @param configPath The path to the (JSON) config file.
 */
void loadFromConfigFile(string configPath) {
	if (configPath == "default") configPath = jocsConfigPath;

	ifstream configStream(configPath);

	if (!configStream) {
		cout << "Unable to read config file" << endl;
		return;
	}

	/// Parse the config file
	std::string configData((std::istreambuf_iterator<char>(configStream)), std::istreambuf_iterator<char>());
	nlohmann::json rawJson = nlohmann::json::parse(configData);
	loadJocsFile(rawJson);
}

void socket_on_message(const json &data) {

	string type = data["type"];

	if (type == "prepare") {
		printf("Preparing...\n");
		prepareMode = true;
	} else if (type == "play") {
		printf("Playing...\n");
		playMode = true;
	} else if (type == "pause") {
		printf("Pausing...\n");
		pauseMode = true;
	} else if (type == "stop") {
		printf("Stopping...\n");
		stopMode = true;
	} else if (type == "reset") {
		printf("Resetting...\n");
	} else if (type == "list"){
		printf("Sending file list...\n");
		send_file_list();
	} else if (type == "load"){
		printf("Loading jocs file...\n");
		loadMode = true;
		loadJocsFile(data);
	} else if (type == "kill") {
		bool enabled = data["enabled"];
		printf("Killing...\n");
		killmode = enabled;
	} else {
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

/*
	Must be called when holding in singleDrone.jocs
	TODO: Provide some distance measure during holding phase to determine if the drone is stable enough to calibrate
*/
void do_calibrate() {
	if(vehicles.size() != 1) {
		cout << "Can only calibrate one vehicle at a time" << endl;
		return;
	}

	if(!player->isReady()) {
		cout << "Must be holding to start calibration" << endl;
		return;
	}

	cout << "Calibrating..." << endl;

	double sum = 0.0;
	Rate r(10);
	for(int i = 0; i < 40; i++) {
		sum += vehicles[0]->lastRawControlInput.z();
	}

	sum /= 40.0;

	string calibId = useMocap? to_string(vconfigs[0].net_id) : "sim";

	cout << "Done!" << endl;
}

pthread_t console_handle;

void *console_thread(void *arg) {
	while (running) {
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

		if (args[0] == "prepare") {
			cout << "Preparing..." << endl;
			prepareMode = true;
		} else if (args[0] == "play") {
			cout << "Playing..." << endl;
			playMode = true;
		} else if (args[0] == "pause") {
			cout << "Pausing..." << endl;
			pauseMode = true;
		} else if (args[0] == "stop") {
			cout << "Stopping..." << endl;
			stopMode = true;
		} else if (args[0] == "kill") {
			killmode = args.size() <= 1 || !(args[1] == "off");
		} else if (args[0] == "load" && args.size() > 1) {
			cout << "Loading..." << endl;
			loadMode = true;
			loadFromConfigFile(args[1]);
		}
		else if(args[0] == "calibrate") {
			do_calibrate();
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
	useMocap = rawJson["useMocap"];
	bool enableMessaging = rawJson["enableMessaging"];
	bool enableOSC = rawJson["enableOSC"];

	if (useMocap) {
		nlohmann::json hardwareConfig = rawJson["hardwareConfig"];
		config.clientAddress = hardwareConfig["clientAddress"];
		config.serverAddress = hardwareConfig["serverAddress"];
	}

	tansa::init(enableMessaging);

	if (enableMessaging) {
		tansa::on_message(socket_on_message);
	}

	// Only pay attention to homes of active drones
	// TODO: Have a better check for mocap initialization/health
	if (useMocap) {
		mocap = new Mocap();
		mocap->connect(config.clientAddress, config.serverAddress);
	} else {
		gazebo = new GazeboConnector();
		gazebo->connect();
	}

	if (enableOSC) {
		OSC *osc = new OSC();
		osc->start(53100);
		osc->set_listener(osc_on_message);
	}

	player = new JocsPlayer();

	int i = 0;

	/*
	// For sample lighting demo
	float level = 0;
	float dl = 0.005;
	*/

	signal(SIGINT, signal_sigint);
	running = true;
	printf("running...\n");

	console_start();

	Rate r(100);
	while (running) {
		// Regular status messages
		if (enableMessaging && i % 20 == 0) {
			send_status_message();
		}

		if (killmode) {
			for(Vehicle *v : vehicles)
				v->terminate();
		} else if (loadMode) {
			printf("Still loading...\n");
		} else if (player == nullptr || !initialized) {
			// Player not initialized: no-op
		} else if (prepareMode) {
			prepareMode = false;
			player->prepare();
		} else if (playMode) {
			playMode = false;
			player->play();
		} else if (pauseMode) {
			pauseMode = false;
			player->pause();
		} else if (stopMode) {
			stopMode = false;
			player->stop();
		} else {
			player->step();
		}

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
	for (int vi = 0; vi < vehicles.size(); vi++) {
		Vehicle *v = vehicles[vi];
		v->disconnect();
		delete v;
	}

	player->cleanup();

	printf("Done!\n");
}
