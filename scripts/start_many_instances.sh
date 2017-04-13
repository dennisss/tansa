#!/bin/bash
# Based on PX4/Tools/sitl_multiple_run.sh with modifications for starting with simulator etc.


# Number of vehicles to spawn
num=$1
rc_script="config/models/x340/rcS"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Killing running instances"
pkill px4
sleep 1

n=0
while [ $n -lt $num ]; do
	$SCRIPT_DIR/start_instance.sh $n $rc_script &
	n=$(($n + 1))
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
