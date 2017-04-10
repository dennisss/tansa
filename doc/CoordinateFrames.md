Coordinate Frames Explained
===========================

Coordinate frame and rotations can sometimes be confusing, so here we seek to explain which ones we use and later on the derivations of the transformations between these frames.

If you are just a casual user who wants to setup some missions/choreographies for your drones to follow, then you should only need to read the first section of the page.


What We Use Briefly (ENU)
-------------------------

Most regular people outside of research probably think of ENU coordinate frame when they think of 3D coordinate systems. In ENU, thinking of coordinates fixed at some origin on the floor, the +x axis is typically to the right, the +y is forwards and +z is upwards (out of the ground).

Regarding orientations, when the drone is at an angle of '0', it is pointing towards the +x axis. All these frames are right handed and follow right handed rotation rules, so a drone is facing the +y axis, it has a yaw (angle about z axis) of positive 90 degrees.

All computations in this package are done in ENU space. For the most part conversions from other frames used by external systems and flight controllers is performed under the hood in their respective drivers. We use ENU as a global coordinate frame, meaning that the origin never changes, so points map to the positions in space that you'd expect.


What Others Use
---------------

- ROS and Gazebo use ENU
- PX4 uses NED in both a global and body fixed sense


ENU
---

The acronym stands for East, North, Up. In general the acronym indicates the direction of the +x, +y, and +z axes in that order.

Although these are geographical directions, unless using GPS, we typically use it based on right, forward, up of some origin defined by our positioning system.


NED
---

It stands for North East Down. This is what you'll see in PX4 data logs. Likewise, a drone is conventionally pointing in the +x direction  Note that because z is pointing downwards, taking off of the ground means getting a more negative z coordinate, and spinning clockwise (looking down from above) results in a positive angle.


Body Frames
-----------

All these coordinate frames as described so far have their origin fixed at some single point in space as a global frame. But, for onboard algorithms, this is inconvenient because IMUs and motors are moving and rotated, so what they sense/do is rotated by the global transformation.

To help in this, flight controllers such as PX4 define a body-fixed frame which always has a position of (0,0,0) fixed at the drone, and a rotation of 0 with the drone always pointing in +x.

The position pose of the drone in the global frame can then be formulated as the conversion of a point at (0,0,0) in this body-fixed frame into the global frame.


Global Frame Conversions (ENU <-> NED)
--------------------------------------

Because we have to interoperate between other systems using NED, we need a solid set of conversions. We will examine converting ENU to NED, the opposite conversion is to do the inverse of the below operations:

Converting Position:
- As (E, N, U) corresponds to (N, E, D)
	- +x becomes +y  (N in NED is the old +y in ENU)
	- +y becomes +x
	- +z becomes -z
- This results in the code: `pt_ned = ( pt_enu.y, pt_enu.x, -pt_enu.z )`
- Or in this package you will see it as the equivalent matrix multiplication: `pt_ned = [0, 1, 0; 1, 0, 0; 0, 0, -1] * pt_enu`


Converting Orientation
- This is a little trickier as the 'forward' direction changes when flipping the axes like above.
- Considering the above matrix `A`, doing `R_ned = A * R_enu`, if `R_enu` is the identity matrix (no rotation), then this would mean that the drone is North which is not the case as a rotation of 0 in ENU is pointing East
- Separately, we know that if the drone is facing East in NED, this corresponds to a +90 degree z rotation
	- In matrix notation that is `Ri = [0, -1, 0; 1, 0, 0; 0, 0, 0]`
- We solve this by post-multiplying a rotation matrix `B` which performs the initial alignment before additional rotations
- The equation `Ri = A * Identity * B` implies that `B = [1, 0, 0; 0, -1, 0; 0, 0, -1]`
- So in general to convert a rotation, we do `R_ned = A * R_enu * B`
