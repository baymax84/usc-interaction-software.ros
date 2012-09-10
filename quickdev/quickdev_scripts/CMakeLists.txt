cmake_minimum_required( VERSION 2.6 )

include( $ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake )

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set( ROS_BUILD_TYPE RelWithDebInfo )

rosbuild_init()

#file( GLOB_RECURSE scripts_sources ${CMAKE_CURRENT_SOURCE_DIR} quickdev-* )
#install( PROGRAMS ${scripts_sources} DESTINATION /usr/bin )

install( PROGRAMS bash/quickdev-create-pkg bash/quickdev-add-license DESTINATION /usr/bin )
