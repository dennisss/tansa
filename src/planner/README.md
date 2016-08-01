Planner
---

- Converts a plan to a set of trajectories and other commands for the drones to follow
- A plan consists of a single root plan which may also embed inner plans inside of it

- Types of plans
	- ConcurrentPlan : runs multiple plans at the same time (typically a TrackPlan is nested inside of it)
	- TrackPlan : runs multiple sequentially
	- BaseMotionPlan
		- Input: set of position/velocity pairs
		- Update loop: While not at position, send current
		- Termination
	- LinearMotionPlan : Go from the current position to a set of waypoints
		- Feeds plain positions without vectors into the BaseMotionPlan
	- SmoothMotionPlan : The LinearMotionPlan using minimum snap
	- LightControlPlan : Turn on or off one of the LEDs

- Each plan should allow 'muxing' the outputs to different drones
- The GUI will be responsible for enforcing that the same outputs aren't simultaneously being used by more than one drone


Input Format
---

{
	type: 'concurrent',
	children: [
		{
			type: 'track',
			children: [
				{
					type: 'linearMotion',
					points: [
						[0,0,2],
						[1,0,2],
						[1,1,2],
						[0,1,2],
						[0,0,2]
					]
				}
			]
		},
		{
			type: 'track',
		}
	]
}


Outputs
---

- MAV position and velocity
- Rigid body group
	- Model many drones as one center point
- LED state
