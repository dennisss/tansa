Tansa: Dancing Robotics Platform
========================

For making drones dance


Setup
-----

- WiFi Router at 192.168.8.1
	- Drones as static IPs in .8X range

Routine
-------
- A sequence of many planned motions
- All motions are continuous as they
- Types of motions
	- Special Takeoff/Land/Off
	- Line : go from one point to another point
	- Curves : go through many points
		- Using either the Minimum Snap algorithm or Bezier curves
- Ques can be used to trigger events (multiples queues can be set for the same motion)
	- Follows : motion will happen immediately after the previous one was completed (the default)
	- Disturb : when something is about to hit the drone, start the motion





Coordinate Systems
------------------

- OptiTrack Motive (TODO: Check this?)
	- "X (Pitch), Y (Yaw), Z (Roll), Right-Handed (RHS), Relative Axes (aka 'local')"
	- Calibration square placement
		- Long leg (+Z) should be towards front of stage
		- Short leg (+X) should be left

- ROS
	- ENU

- PX4
	- NED





I need to be able to develop and test in simulation the linear motion stuff
- The easy solution is to use linux, but that would be annoying at this point



PX4 Setup
---------

- For motion capture to be used, set ATT\_EXT\_HDG\_M to 2.

roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0

See https://404warehouse.net/2015/12/20/autopilot-offboard-control-using-mavros-package-on-ros/


PX4 Needed Changes
------------------

Description
---

We are using a PixRacer which runs the PX4 drone Firmware


Tasks
---

- Make an application for PX4 that allows have it listen for vehicle_command messages and


- We need to have two PWM outputs and one digital GPIO output

For LEDs, we need to
- See http://dev.px4.io/tutorial-hello-sky.html
- See PX4/src/systemcmds/motor_ramp (https://github.com/PX4/Firmware/blob/master/src/systemcmds/motor_ramp/motor_ramp.cpp) for a good example of raw PWM control
- See http://dev.px4.io/custom-mavlink-message.html for an intro to mavlink messages
- Example of using vehicle_command https://github.com/PX4/Firmware/blob/master/src/drivers/gimbal/gimbal.cpp#L305
- Grab data from type: MAV_CMD_USER_1 (https://pixhawk.ethz.ch/mavlink/)



Limit the RPY rates in the mixer so that the we don't over do the throttle

Motor tester

Propeller tester based on vibration FFT

Local Position Estimator
- Allow Model based deadreckoning (may need to add this to the attitude estimator as well


make posix_sitl_default gazebo

make posix gazebo_iris_opt_flow
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
rosrun dancer offboard_test



1) PX4 always operates in OneShot mode (its a reactive design21). Just set PWM_MIN to 125 and PWM_MAX to 250 to set the right pulse length. It should be accepted by your ESCs then.
