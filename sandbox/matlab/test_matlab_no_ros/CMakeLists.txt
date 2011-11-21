cmake_minimum_required(VERSION 2.4.6)

include( ${CMAKE_CURRENT_SOURCE_DIR}/../matlab/cmake/mex.cmake )
include_mex()

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../matlab/include )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory( src )
