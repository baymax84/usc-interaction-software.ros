#!/bin/bash

quiet_flag=0;

while [ "$1" != "" ]; do
  if [ $1 == "-q" ]; then quiet_flag=1; fi
  shift
done

ip_addr=`ip addr show | grep -v "127.0.0.1" | grep -m1 -oP "inet\s\d+\.\d+\.\d+\.\d+" | sed -e 's:inet\s::g'`
if [ "$ip_addr" == "" ]; then ip_addr=127.0.0.1; fi

export ROS_IP=$ip_addr

if [ $quiet_flag != 1 ]; then echo "ROS_IP set to [" $ROS_IP "]"; fi
