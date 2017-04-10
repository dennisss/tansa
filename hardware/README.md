Hardware
========

This folder contains contains the BOMs, CAD designs, and other related notes on the open designs we developed for running with this project.

Highlights:

- `x340`
	- Our version 1 workhorse. Slightly larger than a DJI F330

- `x260`
	- Our version 2 middle size dancing drone. Half the weight of the `x340` and sports over a 3:1 power to weight ratio and over 15-25 minutes of hover time depending on the configuration.


These are made to run with our custom curated version of `PX4/Firmware`. You can find it in `lib/Firmware/` in this repository. Reflash your Pix- board with it and set the airframe number to the name of the drone (240 or 340) when calibrating in QGroundControl
