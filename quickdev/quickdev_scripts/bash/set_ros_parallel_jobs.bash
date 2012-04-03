#!/bin/bash
export ROS_PARALLEL_JOBS=-j`cat /proc/cpuinfo | grep -c processor`

echo "ROS_PARALLEL_JOBS set to [" $ROS_PARALLEL_JOBS "]"
