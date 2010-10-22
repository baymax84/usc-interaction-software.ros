cmake_minimum_required(VERSION 2.4.6)
project(ARToolKitPlus)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# include directories
include_directories(src)
#include_directories(${PROJECT_SOURCE_DIR}/ARToolKitPlus_2.1.1/sample)
include_directories(include)

# set sources
set(sources
src/MemoryManager.cpp
src/DLL.cpp 
src/librpp/rpp.cpp 
src/librpp/rpp_quintic.cpp 
src/librpp/rpp_vecmat.cpp 
src/librpp/rpp_svd.cpp 
src/librpp/librpp.cpp 
src/extra/Profiler.cpp 
src/extra/FixedPoint.cpp
)

# add libraries
add_library(ARTKP ${sources})

# add executables
#rosbuild_add_executable(Multi ARToolKitPlus_2.1.1/sample/multi/src/main.cpp)
#rosbuild_add_executable(Simple ARToolKitPlus_2.1.1/sample/simple/src/main.cpp)
#rosbuild_add_exectuable(Tools ARToolKitPlus_2.1.1/tools/src/main.cpp)

# link executables with libraries
#target_link_libraries(Multi ARTKP)
#target_link_libraries(Simple ARTKP)
#target_link_libraries(Tools ARTKP)
