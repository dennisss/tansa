Mocap Driver
================

For OptiTrack, the NatNet SDK has been rewritten for Linux

Published topics:
- `/mocap/markers` : 
- `/mocap/bodies/1/pose` : Organized by trackable id
- `/mocap/bodies/1/name`


Updating the source code
- Take a look at `PacketClient.cpp` from the NatNet SDK and diff it with the one we are currently using (NatNet 2.9).
- Integrate all the changes into `ClientCore.cpp`
