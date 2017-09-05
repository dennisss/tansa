Terminology
===========

- Vehicle
	- Currently synonymous with drone, MAV, or anything using MAVLink

- Trajectory
	- A motion in 3d which a vehicle should follow.

- Drone ID
	- A unique identifier used to talk/identify a drone. The ID is based on the port to which the drone is sending messages. See `RealLifeSetup.md`. In simulation, these IDs are ordered from `0-N`

- Action
	- A discrete command (motion, lighting effect, etc.) that a vehicle should execute over some time period

- Routine
	- A set of actions for some number of vehicle tracks
	- They do not have ID information. Rather only indexed or relatively labeled tracks

- JOCS
	- Json Object Choreography Schema
	- Our original file format for defining routines

- Role
	- A number stating which track index in the Routine
