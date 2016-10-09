#!/bin/bash

# Sets up the environment variables used by the Gazebo server and gui for finding models/plugins/meshes


# Assuming this is running from the root directory
BUILD=$(pwd)/build_firmware
SRC=$(pwd)/lib/Firmware

export GAZEBO_PLUGIN_PATH=${BUILD}/build_gazebo:$(pwd)/build/src/gazebo:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SRC}/Tools/sitl_gazebo/Build/msgs/:${BUILD}/build_gazebo

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${SRC}/Tools/sitl_gazebo/models:$(pwd)/config/gazebo/models
export GAZEBO_MODEL_DATABASE_URI=""
