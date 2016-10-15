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

#include "spawn.pb.h"

#include <signal.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <sys/wait.h>

#include <boost/shared_ptr.hpp>

#include <iostream>

typedef const boost::shared_ptr<const tansa::msgs::SpawnRequest> SpawnRequestPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Pose> PosePtr;


namespace gazebo {
	class SwarmPlugin : public WorldPlugin {
	public:
		void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

			node = transport::NodePtr(new transport::Node());
			world = _parent;

			node->Init(world->GetName());
			spawnSub = node->Subscribe("~/spawn", &SwarmPlugin::spawn_callback, this);

		}


		void spawn_callback(SpawnRequestPtr &msg) {

			world->SetPaused(true);

			// Remove all old vehicle models
			std::vector<physics::ModelPtr> models = world->GetModels();
			for(int i = 0; i < models.size(); i++) {
				std::string name = models[i]->GetName();
				if(strncmp(name.c_str(), "vehicle_", 8) == 0) {
					world->RemoveModel(models[i]);
				}
			}


			std::ifstream t("config/gazebo/models/x340/x340.sdf");
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


			for(int i = 0; i < msg->vehicles_size(); i++) {
				const tansa::msgs::SpawnRequest_Vehicle &v = msg->vehicles(i);

				model->GetAttribute("name")->Set("vehicle_" + std::to_string(i));

				std::string p = std::to_string(v.pos().x()) + " " +
								std::to_string(v.pos().y()) + " " +
								std::to_string(v.pos().z()) + " " +
								std::to_string(v.orient().x()) + " " +
								std::to_string(v.orient().y()) + " " +
								std::to_string(v.orient().z());

				pose->Set<std::string>(p);
				port->Set<int>(14561 + 10*i);
				world->InsertModelSDF(file);
			}

			world->SetPaused(false);

			start_sitl(msg->vehicles_size());


		}


		void stop_sitl() {
			if(sitl_process == 0)
				return;

			kill(sitl_process, SIGINT);
			waitpid(sitl_process, NULL, 0);
		}

		void start_sitl(int n) {
			int p = fork();
			if(p == 0) { // Child
				char *const argv[] = { "/bin/bash", "scripts/start_sim.sh", "1", NULL};

				execv("/bin/bash", argv);

				exit(0);
				return;
			}

			sitl_process = p;
		}


	private:

		transport::NodePtr node;
		transport::SubscriberPtr spawnSub;
		physics::WorldPtr world;

		transport::SubscriberPtr world_sub;

		int sitl_process = 0;

	};

	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(SwarmPlugin)
}
