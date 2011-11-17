/***************************************************************************
 *  include/quickdev/console.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of usc-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef QUICKDEVCPP_QUICKDEV_CONSOLE_H_
#define QUICKDEVCPP_QUICKDEV_CONSOLE_H_

#include <ros/console.h>
#include <sstream>

#define PRINT_INFO( args... ) ROS_INFO( args )
#define PRINT_WARN( args... ) ROS_WARN( args )
#define PRINT_ERROR( args... ) ROS_ERROR( args )
#define PRINT_DEBUG( args... ) ROS_DEBUG( args )

#define MAKE_STREAM( stream_name, args... ) \
std::stringstream stream_name; \
stream_name << args << std::endl

#define STREAM_TO_STRING( stream_name ) \
stream_name.str().c_str()

#define STREAM_INFO( args... ) \
MAKE_STREAM( ss, args ); \
PRINT_INFO( "%s", STREAM_TO_STRING( ss ) )

#define STREAM_WARN( args... ) \
MAKE_STREAM( ss, args ); \
PRINT_WARN( "%s", STREAM_TO_STRING( ss ) )

#define STREAM_ERROR( args... ) \
MAKE_STREAM( ss, args ); \
PRINT_ERROR( "%s", STREAM_TO_STRING( ss ) )

#define STREAM_DEBUG( args... ) \
MAKE_STREAM( ss, args ); \
PRINT_DEBUG( "%s", STREAM_TO_STRING( ss ) )

#endif // QUICKDEVCPP_QUICKDEV_CONSOLE_H_
