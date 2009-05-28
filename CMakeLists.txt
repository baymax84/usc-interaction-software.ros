cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(bandit)
rospack_add_executable(bandit_node bandit.cpp)
rospack_add_executable(bandit_mover bandit_mover.cpp)

