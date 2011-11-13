###########################################################################
#  cmake/mex.cmake
#  --------------------
#
#  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of usc-ros-pkg nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

set( MATLAB_DIR $ENV{MATLAB_DIR} ) #/usr/local/MATLAB/R2011a )
set( OS $ENV{OS} ) #glnx )
set( ARCH $ENV{ARCH} ) #a64 )

include_directories( ${MATLAB_DIR}/extern/include ${MATLAB_DIR}/simulink/include )
link_directories( ${MATLAB_DIR}/bin/${OS}${ARCH} )

set( CMAKE_C_FLAGS ${CMAKE_C_FLAGS} -DMATLAB_MEX_FILE -ansi -D_GNU_SOURCE -fPIC -fno-omit-frame-pointer-pthread -DMX_COMPAT_32 -O -DNDEBUG )
set( CMAKE_C_FLAGS ${CMAKE_C_FLAGS} -Wl,--version-script,${MATLAB_DIR}/extern/lib/${OS}${ARCH}/mexFunction.map -Wl,--no-undefined )
set( CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x )

set( MATLAB_LIBS mx mex mat m )

macro( rosbuild_add_mex lib )
  rosbuild_add_library( ${lib} ${ARGV} )
  target_link_libraries( ${lib} ${MATLAB_LIBS} )
  add_custom_command(
    TARGET ${lib}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${LIBRARY_OUTPUT_PATH}/lib${lib}.so ${PROJECT_SOURCE_DIR}/mex/${lib}.mex${ARCH} )
endmacro( rosbuild_add_mex )
