Server
===

- This will drive the program via the web-based interface

- It can either spin up a simulation instance or connect to a machine running all the physical software


- For running the simulation, we will spin up a Docker image with ROS
	- gazebo_ros
		- Create world file with n number of drones (or add them in after the empty world is made?)
		- Needs to use use_sim_time to ensure correct timing
	- PX4 main_app instances per node





What I need to do
- setup dockerode
	- bind source code volume to container
- run the minimal code needed for starting ros/gazebo/mavros
	- compile tansa (linear plan follower)
- run the driver
