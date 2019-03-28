#!/bin/bash

# usage: source robot_network_config.sh

ROBOT_IP=`hostname -I | cut --delimiter " " --fields 1`

echo "Configuring ROS Networking on Master [IP: $ROBOT_IP]"

# Master only cares about host IP
export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$ROBOT_IP
export ROS_HOSTNAME=$ROBOT_IP

echo "ROS Master Networking configured!"
