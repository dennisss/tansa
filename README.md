Tansa: Dancing Robotics Platform
================================

An open platform for making drones dance! This project includes the following:

1. The reference quadcopter electrical/mechanical design used for developing the system
	- Around $500 BOM cost. Requires a drill, saw, 3d printer, off the shelf parts, and time to replicate.
	- Features 2 controllable spotlights for theatrical purposes

2. Drone control algorithms and drivers
	- Written in C++ with no dependency on ROS
	- Interface to networked MAVLink vehicles
	- Trajectory generation and control
		- Control based on the paper: `Minimum Snap Trajectory Generation and Control for Quadrotors` by Daniel Mellinger and Vijay Kumar
	- Motion capture support
		- Driver for using with OptiTrack Motive 1.9.0

3. Operator User Interface
	- TODO

License
-------

Unless otherwise specified, all current files and future contributions in this repository are distributed under the MIT License (see `LICENSE` file).


Requirements
------------

This platform was primary developed for UNIX based OSes (Linux / Mac OS X), but should be mostly compatible with Windows as well.

For running/compiling the core code, the following libraries must be installed:

- CMake
- Eigen3
- CGAL

For running the simulations, all dependencies on the PX4 toolchain/gazebo SITL environment must be met:
- See http://dev.px4.io/starting-installing.html
- See http://dev.px4.io/simulation-gazebo.html
	- We test with Gazebo 7


Building
--------

In `lib/Firmware`, run `make posix_sitl_default gazebo` to start the simulator.

Alternatively, run `make sim` to start up a multidrone simulator

Run `make run` to build the core stuff and start up the controller


Documentation
-------------

See the `doc` folder
