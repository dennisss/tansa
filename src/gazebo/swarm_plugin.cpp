/*
	Gazebo plugin loaded into an empty world to make it easy add in many distinguishable drone models

	The mavlink interface in sitl_gazebo listens for messages on a random port
	and sends messages to the given config port (default 14560). Likewise the PX4
	simulator module be default listens for messages on 14560
*/

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include <string>
#include <fstream>
#include <streambuf>

#include <iostream>

#include "json.hpp"

using json = nlohmann::json;


namespace gazebo {
	class SwarmPlugin : public WorldPlugin {
	public:
		void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

			std::ifstream ht("config/home.json");
			std::string jstr((std::istreambuf_iterator<char>(ht)), std::istreambuf_iterator<char>());

			json homePositions = json::parse(jstr);


			std::ifstream t("lib/Firmware/Tools/sitl_gazebo/models/iris/iris.sdf");
			std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

			sdf::SDF file;
			file.SetFromString(str);

			sdf::ElementPtr root = file.Root();
			sdf::ElementPtr model = root->GetFirstElement();
			sdf::ElementPtr pose = model->AddElement("pose");

			// Finding a reference to the plugin which handles mavlink
			sdf::ElementPtr plugin = model->GetElement("plugin");
			while(plugin->GetAttribute("name")->GetAsString() != "mavlink_interface") {
				plugin = plugin->GetNextElement("plugin");
			}

			if(!plugin) {
				// Couldn't find it
			}

			sdf::ElementPtr port = plugin->GetElement("mavlink_udp_port");

			for(int i = 0; i < homePositions.size(); i++) {

				model->GetAttribute("name")->Set("vehicle_" + std::to_string(i));

				std::string p = "";
				for(int j = 0; j < 6; j++) {
					p += std::to_string(homePositions[i][j].get<double>()) + " ";
				}

				std::cout << p << std::endl;

				pose->Set<std::string>(p);
				port->Set<int>(14561 + 10*i);
				_parent->InsertModelSDF(file);
			}

		}

	private:

	};

	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(SwarmPlugin)
}
