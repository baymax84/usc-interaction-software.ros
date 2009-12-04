cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE( FLTK REQUIRED )
INCLUDE_DIRECTORIES(${FLTK_INCLUDE_DIR})
LINK_LIBRARIES(${FLTK_LIBRARIES})

rospack(bandit)
rospack_add_executable(bandit_node bandit_node.cpp)
rospack_add_executable(bandit_mover bandit_mover.cpp)
rospack_add_executable(fake_bandit fake_bandit.cpp)

