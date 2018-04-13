#!/bin/bash

IP="192.168.2.$1"

echo $IP

SSH="sshpass -p raspberry ssh pi@$IP"
SCP="sshpass -p raspberry scp pi@$IP:"

$SSH raspistill -o cam.jpg # -ss 12000
sleep 1
$SCP~/cam.jpg ./cam$1.$2.jpg
