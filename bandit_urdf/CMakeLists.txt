cmake_minimum_required(VERSION 2.6.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_executable(parser src/parser.cc)
target_link_libraries(parser freeimage)

