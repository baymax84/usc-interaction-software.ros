cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE( FLTK REQUIRED )
INCLUDE_DIRECTORIES(${FLTK_INCLUDE_DIR})
LINK_LIBRARIES(${FLTK_LIBRARIES})

rosbuild_init()
rosbuild_add_executable(bandit_node bandit_node.cpp)
rosbuild_add_executable(bandit_mover bandit_mover.cpp)
rosbuild_add_executable(fake_bandit fake_bandit.cpp)

