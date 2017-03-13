Mocap Driver
================

For OptiTrack, the NatNet SDK has been rewritten for Linux

Use the tansa::Mocap class and add Vehicles to it


Updating the source code
- Take a look at `PacketClient.cpp` from the NatNet SDK and incorporate any changes to the data stream
	- This driver is compatible with NatNet 2.9.
- Integrate all the changes into `ClientCore.cpp`
