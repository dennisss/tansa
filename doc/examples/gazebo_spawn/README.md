Gazebo Spawning Example
=======================

This example shows how to spawn 2 drones in a gazebo simulation at some positions.

1. In the root directory of this repository compile the main libraries using:
	- `make build`
	- `make build_firmware`

2. Also in the root, start an instance of the gazebo world using `make runSim`

3. In this directory, run `cmake .` and `make`

4. You can now run `./spawner` and two vehicles should spawn in the world

See `main.cpp` for usage.
