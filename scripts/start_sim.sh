#!/bin/bash
# Based on PX4/Tools/sitl_multiple_run.sh with modifications for starting with simulator etc.

# Number of vehicles to spawn
num=6

sim_port=14561

main_port=14555 # Used to Identify the drone
main_porto=14550 # Held constant for all drones

dbg_port=14551
dbg_porto=14556

port_step=10

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
root_path="$SCRIPT_DIR/.." # Root of this repository

rc_script="config/gazebo/iris"
tmp_path=${root_path}/tmp

echo "Killing running instances"
pkill px4
sleep 1



echo "Starting Gazebo"
source ${root_path}/scripts/setup_gazebo_paths.sh

gzserver ${root_path}/config/gazebo/worlds/empty.world &
SIM_PID=`echo $!`

if [[ ! -n "$HEADLESS" ]]; then
	gzclient --verbose &
	GUI_PID=`echo $!`
fi




mkdir -p $tmp_path
cd $tmp_path

user=`whoami`
n=0
while [ $n -lt $num ]; do
	working_dir="vehicle_$n"
	if [ ! -d $working_dir ]; then
		mkdir -p "$working_dir"
		pushd "$working_dir" &>/dev/null

		# replace template config with configured ports of current instance
		cat ${root_path}/${rc_script} | sed s/_SIMPORT_/${sim_port}/ | \
			sed s/_MAINPORT_/${main_port}/g | sed s/_MAINPORTO_/${main_porto}/ | \
			sed s/_DBGPORT_/${dbg_port}/ | sed s/_DBGPORTO_/${dbg_porto}/ > rcS
		popd &>/dev/null
	fi

	pushd "$working_dir" &>/dev/null
	echo "Starting instance #$n in $(pwd)"
	sudo -b -u $user ${root_path}/build_firmware/src/firmware/posix/px4 -d "$root_path/lib/Firmware" rcS >out.log 2>err.log
	popd &>/dev/null

	n=$(($n + 1))

	sim_port=$(($sim_port + $port_step))
	main_port=$(($main_port + $port_step))
	main_porto=$(($main_porto + $port_step))

	dbg_port=$(($dbg_port + $port_step))
	dbg_porto=$(($dbg_porto + $port_step))
done


# Wait until interrupted
echo "Press Ctrl-C to terminate"
I=0
trap 'I=1' INT
while [ $I -lt 1 ]
do
	sleep 0.5
done

echo "Killing everything"


pkill px4
sleep 1

kill -9 $SIM_PID
if [[ ! -n "$HEADLESS" ]]; then
	kill -9 $GUI_PID
fi
