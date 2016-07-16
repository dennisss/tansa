Multi-drone manager
===================

- Bridges the gap between the rigid bodies published by mocap and mavros
	- mocap will publish a raw unsorted list of rigid bodies. Then, we will actuate
- A configuration file specifies each drone in the network, how to connect to them and which type they are

Published Topics
----------------

- `/swarm/1/mavros` : Raw access to the mavros for the drone
