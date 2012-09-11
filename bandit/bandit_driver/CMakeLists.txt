cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

FIND_PACKAGE( FLTK REQUIRED )
INCLUDE_DIRECTORIES(${FLTK_INCLUDE_DIR})
LINK_LIBRARIES(${FLTK_LIBRARIES})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory( src )