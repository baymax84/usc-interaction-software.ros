cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE( FLTK REQUIRED )
INCLUDE_DIRECTORIES(${FLTK_INCLUDE_DIR})
LINK_LIBRARIES(${FLTK_LIBRARIES})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


rosbuild_add_executable(bandit_node bandit_node.cpp)
#rosbuild_add_executable(bandit_mover bandit_mover.cpp)
#rosbuild_add_executable(bandit_calib bandit_calib.cpp)
#rosbuild_add_executable(bandit_calib_version1 bandit_calib_version1.cpp)
#rosbuild_add_executable(bandit_yaml_parser bandit_yaml_parser.cpp)
rosbuild_add_executable(bandit_demo_gestures bandit_demo_gestures.cpp)
