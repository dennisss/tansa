Optical Motion Capture Camera Onboard Software
===============================================

This is the software that would run a single motion capture camera.

It is a networked node in the mocap camera network and is responsible for:

- Configuring the physical (or virtual) camera

- Capturing a grayscale image from that camera

- Optionally doing onboard processing of the image data into blobs
	- TODO: Maybe also undistort the points?

- Send the frame data over the network

- TODO: Allow support for onboard IMUs for assistance with alignment
	- The IMU would need to be well attached (oriented exactly parallel to the camera so that there isn't any alignment issues)



Raspberry Pi
------------

Currently this is designed to run on a Raspberry Pi with a No-IR Camera board v2

- 1640x1232 resolution will offer the full FOV at up to 40FPS
- Best to use a Raspberry Pi 3 or the latest compute module with a quad core
- Minimum 4GB system disk
- Should run a copy of Rasbian
 	- Install with a pre-emptive kernel maybe?
	- Give the GPU 128Mb of memory
	- Have the CPU frequency looked to the maximum all the time (while running)
- Should also have a copy of ptpd running for time synchronization


TODO:
- Boost CPU to at least 1.35GHz
- Memory boost to 500MHz (stock is 400MHz)
	- See http://www.jackenhack.com/raspberry-pi-3-overclocking/



Virtual
-------






A virtual camera defined by:

- Pose (6D)
- Resolution
- FOV (defines the focal length)
- Frame rate

- Exposure
	- Measured in microseconds
	- In general our exposure will be small compared to the simulation integration interval, so we will literally need to apply a first order approximation over the interval
		- Depending on the speed and distance to the camera, we can dynamically select how many intermediate frames can be generated and blended to make the final image

- Threshold
- IR light emitter intensity



On every simulation update:
- Use OpenGL to render all quadcopter meshes and marker spheres
	- Model propellers are semi-transparent dark discs
