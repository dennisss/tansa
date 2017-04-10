Real Life Setup
===============

A general overview of all the hardware components of the system can be found below. This describes the general configuration needed to get the system to work well with a motion capture system.

![](system-overview.png)


Computers
---------

We use two computers:

- For portability, we use two laptops

- The first runs Ubuntu 16.04 Linux (or Mac when developing routines) and runs all of Tansa

- A second one runs Windows 10 with OptiTrack Motive
	- While it is tempting to replace this computer with a VM, we've had a lot of latency issues with this and found that running as a native machine is much better.
	- We NEVER connect this computer to the internet as we don't want updates to interfere with the stability of the system
	- This computer needs 2 ethernet ports
		- If a laptop, Macbook Pro 15's have 2 thunderbolt ports which work well for this


Networking
----------

- WiFi router
	- We use an `ASUS AC1750 (RT-AC66U)`
	- Setup to `192.168.1.*` subnet
	- Disable router firewall and any packet prioritization features
	- Change router DHCP server to only assign in the range 192.168.1.100 to 192.168.1.254
		- This way a computer can not collide ips with a drone

- Linux Laptop
	- Connected via ethernet to the router

- Windows Laptop
	- Connected via ethernet to the router
	- Separately connected via ethernet to the motion capture switch

- Each drone
	- Currently we support support any drone accessible on a MAVLink UDP port
	- We use MavESP8266
		- Setup in STA mode with a static IP
		- IP: `192.168.1.(80 + X)` where X is the drone id
		- Hport set to `14550 + X*10` for each drone


Motion Capture Setup
--------------------

When calibrating the motion system, the below coordinate system should be marked with tape on the ground. This will represent the (0,0) origin for all maneuvers performed. Note that +Y is along the long end of the CS200 calibration square. When defining the rigid bodies in Motive, make sure that the drone is pointing in the calibration direction specified below.

![](coordinate-frame.jpg)


When setting up OptiTrack Motive, the following settings should be used. Please note that the program is set to unicast and streams both rigid bodies, labeled, and unlabeled markers.

- Additionally, make sure that the interface is set to the one in the `192.168.1.*` range


![](motive-settings.png)


Kill Switch
-----------

We recommend putting FrSky receivers on every single drone. They can all be simultaneously bound to a single transmitter. If an issue occurs, we can kill them all at once.
