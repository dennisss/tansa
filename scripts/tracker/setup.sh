#!/bin/bash
# Initial setup script for Raspberry Pi based trackers
# Given that each Pi has a clean copy of Raspbian Lite:
# 1. Manually ssh in and expand the sd card
# 2. Run this script once on an external computer as: (UNIQUE_NUMBER should by 1,2,3,..)
#    ./scripts/tracker/setup.sh IP_ADDRESS_OF_PI UNIQUE_NUMBER
# Then reboot the raspberry pi

# All the commands needed to setup a Raspberry Pi to have all of the tracker software
# Run this command as ''

# TODO: Eventually use something like https://hub.docker.com/r/sdthirlwall/raspberry-pi-cross-compiler/ for building the program

IP_ADDRESS="$1"
NUM="$2"

HOSTNAME="tracker-$2"
SSH_KEY="~/.ssh/id_tracker"

# Command to use for accessing the Pi once the ssh key is in place
SSH="ssh -o IdentitiesOnly=yes -F /dev/null -i "$SSH_KEY" pi@$IP_ADDRESS"

# If this is the first time that this has been run, then we need to make an ssh key
# The same ssh key will be used for all trackers
if [ ! -f "$SSH_KEY" ]; then
	echo "* Creating new ssh key for trackers"
	ssh-keygen -f "$SSH_KEY" -t rsa -N ''
fi

echo "* Copying ssh key (enter the original password: 'raspberry'):"
cat "$SSH_KEY" | ssh pi@$IP_ADDRESS 'umask 0077; mkdir -p .ssh; cat >> .ssh/authorized_keys && echo "- Copied!"'


echo "* Changing host name"
$SSH echo "$HOSTNAME" > /etc/hostname
$SSH sed -i "s/raspberrypi/$HOSTNAME/g" /etc/hosts


echo "* Installing build dependencies"
$SSH sudo apt-get update
$SSH sudo apt-get install -y build-essential git cmake libeigen3-dev

# TODO: Also install libjpeg-turbo
# Until we work out all the openmx an mmal stuff, we'll just use the generic drivers such as those

echo "* Installing raspicam"
$SSH "
cd ~
git clone https://github.com/dennisss/raspicam .
cd raspicam
mkdir build
cd build
cmake ..

make
sudo make install
sudo ldconfig
"


echo "* Cloning initial repository"
$SSH git clone https://github.com/dennisss/tansa; git checkout mocap


echo "* Building initial camera software"
$SSH cd tansa; make build
# TODO: Make a custom CMake target for the raspberry pi stuff


# https://www.dexterindustries.com/howto/run-a-program-on-your-raspberry-pi-at-startup/
echo "* Registering service"
cat "scripts/tracker/tracker.service" | $SSH 'sudo cat >> /lib/systemd/system/tracker.service && echo "- Copied!"'
$SSH "
sudo chmod 644 /lib/systemd/system/tracker.service
sudo systemctl daemon-reload
sudo systemctl enable sample.service
sudo reboot
"

echo "* Done!"
