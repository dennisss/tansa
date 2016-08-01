Multi-drone manager
===================

- Bridges the gap between the rigid bodies published by mocap and mavros


Input
---
- Configuration file for drones
	- For each drone specified we need:
		- The IP/port to connect with mavros
		- Physical drone description
			- List of motion capture dot positions + whether or not they are active/non-active
				- Using these, we will perform ICP between the mocap data and the known rigid body orientation (or we can edit the motive rigid body descriptions to do this correctly)
			- Bounding box
- General Options
	- Simulating : If enabled:
		- gazebo is started with a blank world (one floor plane)
			- n models are added at configured home positions
			- The names of these models will be of the form 'modeltype+number', i.e. 'iris1'
		- start n px4 sitl instances attached to gazebo
			- starting with mavros
		- start 1 simulated mocap instance which uses the model names to redirect mocap data to the appropriate mavros instance


Published Topics
---

- `/swarm/1/mavros` : Raw access to the mavros for the drone
- `/swarm/1/state` : Summary of pose, whether or not it is connected, and if it is registered


Services
---

- `/swarm/register` : For non-simulation mode, this will:
	1. Record all non-rigid body markers located in the scene
	2. Flash the IR LEDs on the drone
	3. Redo 1 and note and location of the newly appearing marker
	4. Find nearest rigid body and register that one to the current drone being pinged







General strategy for running a simulation
---

- Start docker container
- Compile the script
- Launch swarm node (with simulation mode and drone configuration)
	- this will internally start gazebo_ros without gui with an appropriate world
	- setup rootfs and rcS for px4
		- then it will spin up many mainapp instances


	mkdir -p $build_path/src/firmware/posix/rootfs/fs/microsd
	mkdir -p $build_path/src/firmware/posix/rootfs/eeprom
	touch $build_path/src/firmware/posix/rootfs/eeprom/parameters

	cd to xxxx/firmware/posix
	./mainapp ../../../../${rc_script}_${program}_${model}






	cd $n

	  mkdir -p rootfs/fs/microsd
	  mkdir -p rootfs/eeprom
	  touch rootfs/eeprom/parameters

	  cp ${src_path}/ROMFS/px4fmu_common/mixers/quad_w.main.mix ./
	  cat ${src_path}/${rc_script}_gazebo_iris | sed s/_SIMPORT_/${sim_port}/ | sed s/_MAVPORT_/${mav_port}/g | sed s/_MAVOPORT_/${mav_oport}/ | sed s/_MAVPORT2_/${mav_port2}/ | sed s/_MAVOPORT2_/${mav_oport2}/ > rcS
	  cd ../
	 fi

	 cd $n

	 sudo -b -u $user ../mainapp -d rcS >out.log 2>err.log





	-


- roslaunch px4 mavros_sitl_posix.launch
	- Need to make sure that PX4 is compiled and sitl_gazebo has ground_truth enabled
- Main program will wait for gazebo and px4 to start up
- Then it will execute the commands


- Forward the 'gazebo/ModelStates' to mavros
- We should send PoseStamped messages to the '/mavros/mocap/pose topic in order to simulate the mocap
