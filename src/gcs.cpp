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

#include <boost/filesystem.hpp>

using namespace std;
using namespace tansa;
namespace fs = boost::filesystem;

// Working directory for configuration and choreo files
static string defaultWorkspaceDir = ".";
static string workspaceDir;

// All these are relative to the workspace dir
static string paramsDir = "config/params/";
static string dataDir = "data/";
static string settingsPath = "config/config.json";
static string jocsConfigPath = "config/jocs.json";


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

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}

string resolvePath(string a, string b = "", string c = "", string d = "") {
	fs::path p(a);

	if(b != "") p = p / fs::path(b);
	if(c != "") p = p / fs::path(c);
	if(d != "") p = p / fs::path(d);

	return p.string();
}

// Gets the path in the current workspace (used for reading directories and writing files to )
string resolveWorkspacePath(string a, string b = "", string c = "") {
	return resolvePath(workspaceDir, a, b, c);
}

// Trys to find the path either in the current workspace or in the default one (used for reading files with a graceful fallback to defaults)
string searchWorkspacePath(string a, string b = "", string c = "") {
	string cur = resolveWorkspacePath(a, b, c);
	string dflt = resolvePath(defaultWorkspaceDir, a, b, c);

	// If nothing in current workspace, but there is a default, use the default
	if(fs::exists(dflt) && !fs::exists(cur)) {
		return dflt;
	}

	return cur;
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

		jsonVehicle["battery"] = {
			{"voltage", vehicles[i]->battery.voltage},
			{"percent", vehicles[i]->battery.percent}
		};

		jsonVehicles.push_back(jsonVehicle);
	}
	jsonStatus["vehicles"] = jsonVehicles;

	json globalStatus;
	globalStatus["playing"] = player->isPlaying();
	globalStatus["initialized"] = initialized;
	globalStatus["ready"] = player->isReady();
	globalStatus["paused"] = player->isPaused();
	globalStatus["landed"] = player->isLanded();
	globalStatus["time"] = player->currentTime();
	jsonStatus["global"] = globalStatus;

	tansa::send_message("status", jsonStatus);
}


/**
 * Return breakpoint information for a given jocsFile
 * @param jocsPath	Path to a jocs file.
 * @return	json array of breakpoint objects
 */
json getBreakpoints(string jocsPath) {
	json jsonBreakpoints = json::array();
	try {
		auto jocsData = Jocs::Parse(jocsPath, 1.0);
		auto breakPoints = jocsData->GetBreakpoints();

		for (auto &breakPoint : breakPoints) {
			json jsonBreakpoint;
			jsonBreakpoint["name"] = breakPoint.GetName();
			jsonBreakpoint["number"] = breakPoint.GetNumber();
			jsonBreakpoint["start"] = breakPoint.GetStartTime();
			jsonBreakpoints.push_back(jsonBreakpoint);
		}
	} catch (std::runtime_error e) {
		cout << "Encountered an exception in getBreakpoints(" << jocsPath << "): ";
		std::cerr << e.what() << std::endl;
	}

	return jsonBreakpoints;
}

void send_file_list() {
	json j;

	json files = json::array();
	json jsonMessage = json::array();
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir(resolveWorkspacePath(dataDir).c_str())) != NULL) {
		while ((ent = readdir(dir)) != NULL) {
			if(ent->d_type == DT_REG && std::string(ent->d_name).find(".jocs") != std::string::npos)
				files.push_back(std::string(ent->d_name));
		}
		closedir (dir);
	} else {
		generateError(j, "Could not open directory.");
	}

	for (string file : files) {
		json message;
		message["name"] = file;
		message["breakpoints"] = getBreakpoints(resolveWorkspacePath(dataDir, file));
		jsonMessage.push_back(message);
	}

	j["files"] = jsonMessage;
	tansa::send_message("list_reply", j);
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
		vehicles[i]->read_params(searchWorkspacePath(paramsDir, "default.json"));
		string calibId = useMocap? to_string(v.net_id) : "sim";

		if(!vehicles[i]->read_params(searchWorkspacePath(paramsDir, calibId + ".calib.json"))) {
			cout << "ID: " + calibId + " not calibrated!" << endl;
		}

		vehicles[i]->forward(v.lport + 2, v.rport + 2);
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
	tansa::send_message("load_reply", j);
}

// TODO: This is a bad name as it actually uses the config file as the rawJson
/**
 * Load a Jocs file with the configuration parameters specified via either the REPL or the GUI
 * @param rawJson 	JSON containing configuration information to initialize (or reconfigure) the JocsPlayer
 */
void loadJocsFile(const json &rawJsonArg) {
	json rawJson = rawJsonArg;

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

	// Load the default config file
	ifstream defaultConfigStream(searchWorkspacePath(jocsConfigPath));
	std::string defaultConfigData((std::istreambuf_iterator<char>(defaultConfigStream)), std::istreambuf_iterator<char>());
	nlohmann::json defaultRawJson = nlohmann::json::parse(defaultConfigData);

	// Use default vehicles if the gui didn't send it
	if(rawJson.count("vehicles") == 0) {
		rawJson["vehicles"] = defaultRawJson["vehicles"];
	}

	if(rawJson.count("jocsActiveIds") == 0) {
		rawJson["jocsActiveIds"] = defaultRawJson["jocsActiveIds"];
	}


	initialized = false;
	string jocsPath = rawJson["jocsPath"];
	vector<unsigned> jocsActiveIds = rawJson["jocsActiveIds"];
	scale = rawJson["theaterScale"];


	int startPoint = rawJson["startPoint"];
	player->cleanup();
	player->loadJocs(searchWorkspacePath(jocsPath), scale, jocsActiveIds, startPoint);
	vector<Point> homes = player->getHomes();
	spawnVehicles(rawJson, homes, jocsActiveIds);

	// int cue = data["cue"];
	// TODO: Prepare the jocs file to start at this cue.
	// TODO: Need to make sure this gets deleted. Will have to delete inside the JocsPlayer class when we load a new file.
	// In other words, we transfer ownership of the jocs object to the player here.

	initialized = true;
	loadMode = false;
	printf("Finished loadJocsFile\n");
}

/**
 * Legacy method to load entirely from a config file, allows for initialization/loading via the REPL
 * @param configPath The path to the (JSON) config file.
 */
void loadFromConfigFile(string configPath) {
	if (configPath == "default") configPath = searchWorkspacePath(jocsConfigPath);

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
	vehicles[0]->params.hoverPoint = sum;
	vehicles[0]->write_params(resolveWorkspacePath(paramsDir, calibId + ".calib.json"));

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
		} else if (args[0] == "land") {
			printf("landing...\n");
			player->land();
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

	// Configure data and configuration folders
	char *wpath = getenv("TANSA_PATH");
	if(wpath != NULL)
		workspaceDir = wpath;
	else
		workspaceDir = defaultWorkspaceDir;


	// TODO: Show a usage message instead
	assert(argc == 2);
	string configPath = argv[1];

	// TODO: REad configpath fro workspace?
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
