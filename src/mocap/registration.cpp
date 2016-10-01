

/*
	Code for registering the pose of the vehicles based on known marker configurations and active IR beacons

	For vehicles that have just started to be tracked, brute force match marker correspondences
	- Center on centroid
	- Pick one of the 5 or so known markers
		- Propose one the other ones to match to
		- Find best alignment
*/
