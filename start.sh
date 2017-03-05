#!/bin/bash

# Quickstart script

HEADLESS=1 make runSim &
./run.js &
make runGcs
