#include <tansa/action.h>
#include <tansa/control.h>
#include <tansa/core.h>
#include <tansa/config.h>
#include <tansa/jocsPlayer.h>
#include <tansa/mocap.h>
#include <tansa/gazebo.h>
#include <tansa/simulate.h>
#include <tansa/osc.h>
#include <tansa/csv.h>
#include "manager/preview.h"
#include "manager/calibration.h"

#ifdef  __linux__
#include <sys/signal.h>
#endif
#include <signal.h>
//TODO check if these work on OSX
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace tansa;

// Working directory for configuration and choreo files
static string defaultWorkspaceDir = ".";
static string workspaceDir;

// All these are relative to the workspace dir
static string paramsDir = "config/params/";
static string dataDir = "data/";
static string settingsPath = "config/settings.json";
static string routineConfigPath = "config/routine.json";


static bool running = false;
static bool initialized = false;
static bool killmode = false; static int killRole = -1;
static bool terminatemode = false; static int terminateRole = -1;
static bool pauseMode = false;
static bool stopMode = false;
static bool playMode = false;
static bool prepareMode = false;
static bool loadMode = false;
static bool landMode = false;
static float scale = 1.0;
static JocsPlayer* player = nullptr;
static GazeboConnector *gazebo = nullptr;
static Mocap *mocap = nullptr;
static Simulation *sim = nullptr;
static vector<Vehicle *> vehicles;
static std::vector<vehicle_config> vconfigs;
static string worldMode;
static bool inRealLife;
static bool enableLighting;
static int latestStartPoint;

static bool previewMode = false;
static PreviewPlayer *previewPlayer;

static bool calibrationMode = false;
static CalibrationHelper *calibrationHelper;

void signal_sigint(int s) {
	// TODO: Prevent
	running = false;
}

string joinPaths(string a, string b) {
	if(a.size() == 0) {
		return b;
	}
	if(b.size() == 0) {
		return a;
	}

	// TODO: Check that both parts have some length > 0
	if(a[a.size() - 1] == '/') {
		a = a.substr(0, a.length() - 1);
	}
	if(b[0] == '/') {
		b = b.substr(1);
	}

	return a + "/" + b;
}


bool file_exists(const char *file) {
	if(access(file, F_OK) != -1 ) {
		return true;
	}

	return false;
}

bool file_exists(string file) {
	return file_exists(file.c_str());
}


string resolvePath(string a, string b = "", string c = "", string d = "") {
	string p = a;

	if(b != "") p = joinPaths(p, b);
	if(c != "") p = joinPaths(p, c);
	if(d != "") p = joinPaths(p, d);

	return p;
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
	if(file_exists(dflt) && !file_exists(cur)) {
		return dflt;
	}

	return cur;
}


const char *pid_filename = "tmp/tansa.pid";

// For ensuring that there is
void lock_pidfile() {

	ifstream fin(pid_filename);
    if(fin.good()) {
		int oldpid = 0;
		fin >> oldpid;

		if(0 == kill(oldpid, 0)) {
			cout << "Tansa already running. Please close it first." << endl;
			exit(1);
		}
		else {
			cout << "Warning: PID file exists but no process running" << endl;
		}

		fin.close();
	}

	ofstream fout(pid_filename, ofstream::out | ofstream::trunc);
	fout << getpid() << endl;
	fout.flush();
	fout.close();
}

void unlock_pidfile() {
	unlink(pid_filename);
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

void send_preview_status_message() {

	json jsonStatus;

	jsonStatus["type"] = "status";
	jsonStatus["time"] = previewPlayer->get_time();


	std::vector<ModelState> states = previewPlayer->get_states();
	std::vector<int> lightStates = previewPlayer->get_light_states();


	json jsonVehicles = json::array();
	for(int i = 0; i < states.size(); i++) {

		json jsonVehicle;
		jsonVehicle["id"] = i;
		jsonVehicle["role"] = i;
		jsonVehicle["connected"] = false;
		jsonVehicle["armed"] = true;
		jsonVehicle["tracking"] = false;

		jsonVehicle["state"] = previewPlayer->is_playing()? "flying" : "holding";

		json jsonPosition = json::array();
		jsonPosition.push_back(states[i].position.x());
		jsonPosition.push_back(states[i].position.y());
		jsonPosition.push_back(states[i].position.z());
		jsonVehicle["position"] = jsonPosition;

		json jsonOrientation = json::array();
		jsonOrientation.push_back(states[i].orientation.w());
		jsonOrientation.push_back(states[i].orientation.x());
		jsonOrientation.push_back(states[i].orientation.y());
		jsonOrientation.push_back(states[i].orientation.z());
		jsonVehicle["orientation"] = jsonOrientation;

		json lightsArray = json::array();
		if(lightStates.size() > i) {
			int c = lightStates[i];
			lightsArray.push_back( ((c >> 16) & 0xff) / (float)0xff );
			lightsArray.push_back( ((c >> 8) & 0xff) / (float)0xff );
			lightsArray.push_back( ((c >> 0) & 0xff) / (float)0xff );
		}
		jsonVehicle["lights"] = lightsArray;

		json jsonBatteryStats = {
			{"voltage", 8.4},
			{"percent", 1}
		};
		jsonVehicle["battery"] = jsonBatteryStats;

		jsonVehicles.push_back(jsonVehicle);
	}
	jsonStatus["vehicles"] = jsonVehicles;

	json globalStatus;
	globalStatus["mode"] = "preview";
	globalStatus["playing"] = previewPlayer->is_playing();
	globalStatus["ready"] = false;
	globalStatus["paused"] = false;
	globalStatus["landed"] = false;
	jsonStatus["global"] = globalStatus;

	tansa::send_message(jsonStatus);
}


/* For sending a system state update to the gui */
void send_status_message() {

	if(previewMode) {
		send_preview_status_message();
		return;
	}

	if(player == NULL)
		return;

	json jsonStatus;

	jsonStatus["type"] = "status";
	jsonStatus["time"] = player->currentTime();


	std::vector<PlayerVehicleState> states = player->getStates();
	std::vector<unsigned> roles = player->getActiveTracks();

	json jsonVehicles = json::array();
	for(int i = 0; i < vehicles.size(); i++) {
		if(vehicles[i] == NULL) {
			continue;
		}

		json jsonVehicle;
		jsonVehicle["id"] = vconfigs[i].net_id;
		jsonVehicle["role"] = roles.size() > i ? roles[i] : -1;
		jsonVehicle["connected"] = vehicles[i]->connected;
		jsonVehicle["armed"] = vehicles[i]->armed;
		jsonVehicle["tracking"] = vehicles[i]->tracking;

		if(states.size() > i) {
			string s = "unknown";
			switch(states[i]) {
				case StateInit:
					s = "init"; break;
				case StateArming:
					s = "arming"; break;
				case StateReady:
					s = "ready"; break;
				case StateTakeoff:
					s = "takeoff"; break;
				case StateHolding:
					s = "holding"; break;
				case StateFlying:
					s = "flying"; break;
				case StateLanding:
					s = "landing"; break;
				case StateFailsafe:
					s = "failsafe"; break;
			}

			jsonVehicle["state"] = s;
		}

		json jsonPosition = json::array();
		jsonPosition.push_back(vehicles[i]->state.position.x());
		jsonPosition.push_back(vehicles[i]->state.position.y());
		jsonPosition.push_back(vehicles[i]->state.position.z());
		jsonVehicle["position"] = jsonPosition;

		json jsonOrientation = json::array();
		jsonOrientation.push_back(vehicles[i]->state.orientation.w());
		jsonOrientation.push_back(vehicles[i]->state.orientation.x());
		jsonOrientation.push_back(vehicles[i]->state.orientation.y());
		jsonOrientation.push_back(vehicles[i]->state.orientation.z());
		jsonVehicle["orientation"] = jsonOrientation;

		json jsonLights = json::array();
		for(double d : vehicles[i]->lightState)
			jsonLights.push_back(d);
		jsonVehicle["lights"] = jsonLights;

		json jsonBatteryStats = {
			{"voltage", vehicles[i]->battery.voltage},
			{"percent", vehicles[i]->battery.percent}
		};
		jsonVehicle["battery"] = jsonBatteryStats;

		jsonVehicles.push_back(jsonVehicle);
	}
	jsonStatus["vehicles"] = jsonVehicles;

	json globalStatus;
	globalStatus["mode"] = worldMode;
	globalStatus["playing"] = player->isPlaying();
	globalStatus["ready"] = player->isReady();
	globalStatus["paused"] = player->isPaused();
	globalStatus["landed"] = player->isLanded();
	jsonStatus["global"] = globalStatus;

	tansa::send_message(jsonStatus);
}

/**
 * Return breakpoint information for a given jocsFile
 * @param routinePath	Path to a routine file.
 * @return	json array of breakpoint objects
 */
json getBreakpoints(string routinePath) {
	json jsonBreakpoints = json::array();
	try {
		cout << "Load " << routinePath << endl;
		auto r = Routine::Load(routinePath, 1.0);

		if(r == nullptr) {
			json arr = json::array();

			json inv;
			inv["name"] = "Invalid";
			inv["number"] = -1;
			inv["startTime"] = 0;
			arr.push_back(inv);

			return arr;
		}

		auto breakPoints = r->breakpoints;

		for (auto &breakPoint : breakPoints) {
			json jsonBreakpoint;
			jsonBreakpoint["name"] = breakPoint.GetName();
			jsonBreakpoint["number"] = breakPoint.GetNumber();
			jsonBreakpoint["startTime"] = breakPoint.GetStartTime();
			jsonBreakpoints.push_back(jsonBreakpoint);
		}
	} catch (std::runtime_error e) {
		cout << "Encountered an exception in getBreakpoints(" << routinePath << "): ";
		std::cerr << e.what() << std::endl;
	}

	return jsonBreakpoints;
}

void send_file_list() {
	json j;

	j["type"] = "list_reply";
	json files = json::array();
	json jsonMessage = json::array();
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir(resolveWorkspacePath(dataDir).c_str())) != NULL) {
		while ((ent = readdir(dir)) != NULL) {
			if(ent->d_type == DT_REG && Routine::IsFile(std::string(ent->d_name)))
				files.push_back(std::string(ent->d_name));
		}
		closedir (dir);
	} else {
		generateError(j, "Could not open directory.");
	}

	for (string file : files) {
		json message;
		message["fileName"] = file;
		message["breakpoints"] = getBreakpoints(resolveWorkspacePath(dataDir, file));
		jsonMessage.push_back(message);
	}

	j["files"] = jsonMessage;
	tansa::send_message(j);
}

/**
 * Disconnect and delete any existing vehicles, and initialize those specified via JSON
 * @param rawJson 			The JSON object containing the vehicle configuration information (net_id's)
 * @param homes 			Homes for the new vehicles, specified by the current Jocs file
 * @param activeRoles 	The track numbers of the drones to be used in this configuration
 */
void spawnVehicles(const json &rawJson, vector<Point> homes, vector<unsigned> activeRoles) {
	vconfigs.resize(rawJson["vehicles"].size());

	for (unsigned i = 0; i < rawJson["vehicles"].size(); i++) {
		vconfigs[i].net_id = rawJson["vehicles"][i]["net_id"];
		if (inRealLife) {
			vconfigs[i].lport = 14550 + 10*vconfigs[i].net_id;
			vconfigs[i].rport = 14555;
		} else { // The simulated ones are zero-indexed and are always in ascending order
			vconfigs[i].net_id = i;

			vconfigs[i].lport = 14550 + 10*vconfigs[i].net_id;
			vconfigs[i].rport = 14555 + 10*vconfigs[i].net_id;
		}
	}

	// Number of drones used (limited by active ids and number of available slots in choreography)
	int n = activeRoles.size();
	if(homes.size() < n) {
		n = homes.size();
	}

	if (n > vconfigs.size()) {
		printf("Not enough drones on the network\n");
		return;
	}

#ifdef USE_GAZEBO
	gazebo->clear();
#endif

	// Stop all vehicles
	for (int vi = 0; vi < vehicles.size(); vi++) {
		Vehicle *v = vehicles[vi]; vehicles[vi] = NULL;
		v->disconnect();
		delete v;
	}

	vehicles.resize(n, NULL);

	for (int i = 0; i < n; i++) {
		const vehicle_config &v = vconfigs[i];

		vehicles[i] = new Vehicle();

		// Load default parameters
		vehicles[i]->read_params(searchWorkspacePath(paramsDir, inRealLife? "x260.json" : "default.json"));
		string calibId = inRealLife? to_string(v.net_id) : "sim";

		if(!vehicles[i]->read_params(searchWorkspacePath(paramsDir, calibId + ".calib.json"))) {
			cout << "ID: " + calibId + " not calibrated!" << endl;
		}

		vehicles[i]->forward(v.lport + 2, v.rport + 2);
		vehicles[i]->connect(v.lport, v.rport);
		if (inRealLife) {
			mocap->track(vehicles[i], i+1);
#ifdef USE_GAZEBO
		} else if(worldMode == "gazebo") {
			gazebo->track(vehicles[i], i);
#endif
		} else if(worldMode == "sim") {
			sim->track(vehicles[i], i);
		}
	}

#ifdef USE_GAZEBO
	if (worldMode == "gazebo") {
		// Only pay attention to homes of active drones
		vector<Point> spawns;
		for (int i = 0; i < n; i++) {
			int chosenRole = activeRoles[i];
			// We assume the user only configured for valid IDs..
			spawns.push_back(homes[chosenRole]);
			spawns[i].z() = 0;
		}

		// Keeping preview mode fast by not spawning the processes
		if(!previewMode)
			gazebo->spawn(spawns, "config/gazebo/models/x340/x340.sdf", "config/models/x340/rcS");
	}
#endif

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
		json position = json::array();
		position.push_back(home.x());
		position.push_back(home.y());
		position.push_back(home.z());
		positions.push_back(position);
	}
	j["cues"] = nums;
	j["target_positions"] = positions;
	j["duration"] = player->getCurrentFile()->duration();

	json paths = json::array();

	std::vector<std::vector<tansa::Action*>> actions = player->getActions();
	for(int i = 0; i < actions.size(); i++) {
		json pts = json::array();

		double t = 0;
		for(int j = 0; j < actions[i].size(); j++) {

			MotionAction *m = (MotionAction *) actions[i][j];

			Trajectory::Ptr path = m->GetPath();
			for(; t <= m->GetEndTime(); t += 0.1) {

				Vector3d pt = path->evaluate(t).position;
				json jpt = json::array();
				jpt.push_back(pt.x()); jpt.push_back(pt.y()); jpt.push_back(pt.z());

				//if(pts.size() >= 2 && ((pts[pts.size() - 1] - pts[pts.size() - 2]) - (pt - pts[pts.size() - 2])).norm() < 0.001) {

				//}
				//else {
					pts.push_back(jpt);
				//}
			}
		}

		paths.push_back(pts);
	}

	j["paths"] = paths;


	tansa::send_message(j);
}


void sendCalibrationResponse(bool failed) {
	json j;
	j["type"] = "calibrate_reply";
	j["success"] = !failed;

	tansa::send_message(j);
}

/**
 * Load the player from a full configuration object
 * @param rawJson 	JSON containing configuration information to initialize (or reconfigure) the JocsPlayer
 */
void loadConfiguration(const json &rawJsonArg) {
	json rawJson = rawJsonArg;

	loadMode = true;

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
	ifstream defaultConfigStream(searchWorkspacePath(routineConfigPath));
	std::string defaultConfigData((std::istreambuf_iterator<char>(defaultConfigStream)), std::istreambuf_iterator<char>());
	nlohmann::json defaultRawJson = nlohmann::json::parse(defaultConfigData);

	// Use default vehicles if the gui didn't send it
	if(rawJson.count("vehicles") == 0) {
		rawJson["vehicles"] = defaultRawJson["vehicles"];
	}

	// Use default roles if none provided
	if(rawJson.count("activeRoles") == 0) {
		rawJson["activeRoles"] = defaultRawJson["activeRoles"];
	}


	initialized = false;
	string routinePath = rawJson["routinePath"];
	vector<unsigned> activeRoles = rawJson["activeRoles"];
	scale = rawJson["theaterScale"];

	int startPoint = rawJson["startPoint"];
	player->cleanup();

	latestStartPoint = startPoint;

	bool res = false;
	if(routinePath == "custom") {
		res = player->loadChoreography(custom_jocs(), activeRoles, startPoint);
	}
	else {
		res = player->loadChoreography(searchWorkspacePath(routinePath), scale, activeRoles, startPoint);
	}

	if(!res) {
		cout << "Failed to load choreography" << endl;
		loadMode = false;
		return;
	}

	if(rawJson.count("preview") == 1 && rawJson["preview"].get<bool>() == true) {
		previewPlayer = new PreviewPlayer(player->getCurrentFile());
		previewPlayer->play();
		previewMode = true;
	}

	vector<Point> homes = player->getHomes();
	spawnVehicles(rawJson, homes, activeRoles);

	constructLoadResponse();

	initialized = true;
	loadMode = false;
	printf("Finished loadConfiguration\n");
}

/**
 * Legacy method to load entirely from a config file, allows for initialization/loading via the REPL
 * @param configPath The path to the (JSON) config file.
 */
void loadFromConfigFile(string configPath) {
	if (configPath == "default") configPath = searchWorkspacePath(routineConfigPath);

	ifstream configStream(configPath);

	if (!configStream) {
		cout << "Unable to read config file" << endl;
		return;
	}

	/// Parse the config file
	std::string configData((std::istreambuf_iterator<char>(configStream)), std::istreambuf_iterator<char>());
	nlohmann::json rawJson = nlohmann::json::parse(configData);
	loadConfiguration(rawJson);
}

void handle_preview_command(json data) {

}


void do_calibrate();

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
	} else if (type == "seek") {
		if(previewMode) {
			previewPlayer->set_time(data["time"]);
		}
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
		loadConfiguration(data);
	} else if (type == "kill") {
		bool enabled = data["enabled"];
		printf("Killing...\n");

		if(data.count("role") == 1) {
			killRole = data["role"];
		}
		else {
			killRole = -1;
		}

		killmode = enabled;
	} else if (type == "halt") {
		if(data.count("role") == 1) {
			terminateRole = data["role"];
		}
		else {
			terminateRole = -1;
		}
		terminatemode = true;
	} else if (type == "land") {
		printf("Landing...\n");
		landMode = true;
	} else if (type == "calibrate") {
		do_calibrate();
	} else if (type == "resync") {
		if(mocap != NULL) {
			mocap->resync();
		}
	} else if (type == "rearrange") {
		player->rearrange();
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

			if(num < latestStartPoint) {
				printf("Blocking early cue\n");
				return;
			}

			if(player->isPlaying()) {
				printf("Already playing\n");
				return;
			}

			player->play();

			if(mocap != NULL && player->isPlaying()) {
				mocap->start_recording();
			}
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

	calibrationHelper = new CalibrationHelper(vehicles);
	calibrationMode = true;
	return;


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

	string calibId = inRealLife? to_string(vconfigs[0].net_id) : "sim";
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
			killRole = -1;
			killmode = args.size() <= 1 || !(args[1] == "off");
		} else if (args[0] == "load" && args.size() > 1) {
			cout << "Loading..." << endl;
			loadFromConfigFile(args[1]);
		}
		else if(args[0] == "recon"){
			mocap->start_recording();
		}
		else if(args[0] == "recoff") {
			mocap->stop_recording();
		}
		else if(args[0] == "calibrate") {
			do_calibrate();
		}

	}

	return NULL;
}

void console_start() {
	pthread_create(&console_handle, NULL, console_thread, NULL);
}


int main(int argc, char *argv[]) {
	tansa::Context ctx;

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
	if (!configStream) {
		cout << "Unable to read config file" << endl;
		return 1;
	}

	/// Parse the config file
	std::string configData((std::istreambuf_iterator<char>(configStream)), std::istreambuf_iterator<char>());
	nlohmann::json rawJson = nlohmann::json::parse(configData);
	hardware_config config;
	worldMode = rawJson["world"];
	inRealLife = worldMode == "real";
	bool enableMessaging = rawJson["enableMessaging"];
	bool enableOSC = rawJson["enableOSC"];
	bool enableLighting = rawJson["enableLighting"];

	MocapOptions mocap_opts;
	if (inRealLife) {
		nlohmann::json hardwareConfig = rawJson["mocap"];
		mocap_opts.useActiveBeacon = hardwareConfig["useActiveBeacon"];
		if(hardwareConfig["singleDot"]) {
			mocap_opts.mode = MocapRigidBodyFromCloud;
		}
		config.clientAddress = hardwareConfig["clientAddress"];
		config.serverAddress = hardwareConfig["serverAddress"];
	}

	lock_pidfile();

	tansa::init(enableMessaging);

	if (enableMessaging) {
		tansa::on_message(socket_on_message);
	}

	// Only pay attention to homes of active drones
	// TODO: Have a better check for mocap initialization/health
	if (inRealLife) {
		mocap = new Mocap(mocap_opts);
		mocap->connect(&ctx, config.clientAddress, config.serverAddress);
#ifdef USE_GAZEBO
	} else if(worldMode == "gazebo") {
		gazebo = new GazeboConnector();
		gazebo->connect();
#endif
	} else if(worldMode == "sim") {
		sim = Simulation::Make(&ctx); // TODO: Instead run in another thread?
		sim->start();
	}

	if (enableOSC) {
		OSC *osc = new OSC();
		osc->start(53100);
		osc->set_listener(osc_on_message);
	}

	player = new JocsPlayer(inRealLife, enableLighting);

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
		// Regular status messages (currently at 100 / 4 == 25Hz)
		if (enableMessaging && i % 4 == 0) {
			send_status_message();
		}

		if (killmode) {
			player->failsafe(killRole);
			killmode = false;
		} else if (terminatemode) {
			player->terminate(terminateRole);
			terminatemode = false;
		}

		if (loadMode) {
			printf("Still loading...\n");
		} else if (player == nullptr || !initialized) {
			// Player not initialized: no-op
		} else if (prepareMode) {
			prepareMode = false;
			player->prepare();
		} else if (landMode) {
			landMode = false;
			player->land();
		} else if (playMode) {
			playMode = false;

			if(mocap != NULL) {
				mocap->start_recording();
			}

			if(previewMode) {
				previewPlayer->play();
			}
			else {
				player->play();
			}
		} else if (pauseMode) {
			pauseMode = false;
			if(previewMode) {
				previewPlayer->pause();
			}
			else {
				player->pause();
			}
		} else if (stopMode) {
			stopMode = false;

			if(previewMode) {
				previewMode = false;
				delete previewPlayer;
			}
			else {
				player->stop();
			}
		// TODO: Always do a step if possible
		} else if(previewMode) {
			previewPlayer->step();

			if(previewPlayer->ended()) {
				previewMode = false;
				delete previewPlayer;
			}

		} else if(calibrationMode) {
			calibrationHelper->step();

			if(calibrationHelper->done()) {
				// Send message to GUI
				sendCalibrationResponse(calibrationHelper->failed());
				calibrationMode = false;
				delete calibrationHelper;
				cout << "Calibration Done!" << endl;
			}

		} else {
			player->step();
		}

		r.sleep();
		i++;
	}

	/// Cleanup
	if (inRealLife) {
		mocap->disconnect();
		delete mocap;
#ifdef USE_GAZEBO
	} else if(worldMode == "gazebo") {
		gazebo->disconnect();
		delete gazebo;
#endif
	} else if(worldMode == "sim") {
		sim->stop();
		delete sim;
	}

	// Stop all vehicles
	for (int vi = 0; vi < vehicles.size(); vi++) {
		Vehicle *v = vehicles[vi];
		v->disconnect();
		delete v;
	}

	player->cleanup();

	tansa::end();

	unlock_pidfile();

	printf("Done!\n");
}
