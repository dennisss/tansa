#!/bin/bash

# For starting a single instance of the PX4 Firmware given an id and a startup script path
# The rootfs for the instance will be stores in $CWD/tmp/vehicle_$id/
# Call as './scripts/start_firmware.sh id rcs_script_file'

id=$1
rc_script=$2 # Should be something like "config/gazebo/x340"

#### Base port settings ####
port_step=10 # Port interval between ids

sim_port=14561 # The port on which the simulator listens for messages

main_port=14555 # Used to Identify the drone
main_porto=14550 # Held constant for all drones

dbg_port=14551 # Additional mavlink interface
dbg_porto=14556


# Increment each port to get the ones for the current instance
sim_port=$(($sim_port + $port_step*$id))
main_port=$(($main_port + $port_step*$id))
main_porto=$(($main_porto + $port_step*$id))
dbg_port=$(($dbg_port + $port_step*$id))
dbg_porto=$(($dbg_porto + $port_step*$id))




SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
root_path="$SCRIPT_DIR/.." # Root of this repository
tmp_path=${root_path}/tmp

#echo "Killing running instances"
#pkill px4
#sleep 1


mkdir -p $tmp_path
cd $tmp_path

user=`whoami`

working_dir="vehicle_$id"
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
echo "Starting instance #$id in $(pwd)"
sudo -u $user ${root_path}/build_firmware/src/firmware/posix/px4 -d "$root_path/lib/Firmware" rcS >out.log 2>err.log
popd &>/dev/null
