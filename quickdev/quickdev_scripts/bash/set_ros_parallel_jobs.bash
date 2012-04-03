#!/bin/bash
quiet_flag=0;

while [ "$1" != "" ]; do
  if [ $1 == "-q" ]; then quiet_flag=1; fi
  shift
done

export ROS_PARALLEL_JOBS=-j`cat /proc/cpuinfo | grep -c processor`

if [ $quiet_flag != 1 ]; then echo "ROS_PARALLEL_JOBS set to [" $ROS_PARALLEL_JOBS "]"; fi
