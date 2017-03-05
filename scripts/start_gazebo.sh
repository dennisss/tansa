#!/bin/bash

# Starts up a Gazebo server and client using the default empty world and spawning plugin
# This script blocks until a sigint (ctrl-c) is received
# If the enviroment variable HEADLESS=1 is specified, then the official gazebo client is not started

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
root_path="$SCRIPT_DIR/.." # Root of this repository

echo "Starting Gazebo"
source ${root_path}/scripts/setup_gazebo_paths.sh

gzserver ${root_path}/config/gazebo/worlds/empty.world &
SIM_PID=`echo $!`

if [[ ! -n "$HEADLESS" ]]; then
	gzclient --verbose &
	GUI_PID=`echo $!`
fi

# Wait until interrupted
echo "Press Ctrl-C to terminate"
I=0
trap 'I=1' INT
while [ $I -lt 1 ]
do
	sleep 0.5
done

kill -9 $SIM_PID
if [[ ! -n "$HEADLESS" ]]; then
	kill -9 $GUI_PID
fi
