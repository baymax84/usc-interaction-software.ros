cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#SET (PHIDGETIFK_DIR "" CACHE STRING "Directory containing the Phidget IFK headers and libraries")
#MARK_AS_ADVANCED (PHIDGETIFK_DIR)
#IF ("${PHIDGETIFK_DIR}" STREQUAL "")
#    SET (phidgetReqHeader "phidget21.h")
#    SET (phidgetExtraFlags "")
#    SET (phidgetExtraLibs "-lphidget21")
#ELSE ("${PHIDGETIFK_DIR}" STREQUAL "")
#    SET (phidgetReqHeader "${PHIDGETIFK_DIR}/phidget21.h")
#    SET (phidgetExtraFlags "-I${PHIDGETIFK_DIR}/include")
#    SET (phidgetExtraLibs "-L${PHIDGETIFK_DIR}/lib -lphidget21")
#ENDIF ("${PHIDGETIFK_DIR}" STREQUAL "")

genmsg()

INCLUDE_DIRECTORIES(${phidget_SOURCE_DIR}/include)
LINK_LIBRARIES(-L${phidget_SOURCE_DIR}/lib -lphidget21)
rospack_add_executable(phidget phidget.cc)

#target_link_libraries(phidget ${phidgetExtraLibs})


