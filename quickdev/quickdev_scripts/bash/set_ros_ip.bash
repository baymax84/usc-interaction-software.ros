#!/bin/bash
export ROS_IP=`ip addr show | grep -m1 -oP "inet\s192\.\d+\.\d+\.\d+" | sed -e 's:inet\s::g'`

echo "ROS_IP set to [" $ROS_IP "]"
