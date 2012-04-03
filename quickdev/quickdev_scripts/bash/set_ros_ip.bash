#!/bin/bash

quiet_flag=0;

while [ "$1" != "" ]; do
  if [ $1 == "-q" ]; then quiet_flag=1; fi
  shift
done

export ROS_IP=`ip addr show | grep -m1 -oP "inet\s192\.\d+\.\d+\.\d+" | sed -e 's:inet\s::g'`

if [ $quiet_flag != 1 ]; then echo "ROS_IP set to [" $ROS_IP "]"; fi
