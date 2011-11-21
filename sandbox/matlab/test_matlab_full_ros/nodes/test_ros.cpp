/***************************************************************************
 *  nodes/test_ros.cpp
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
 *  * Neither the name of test_matlab_ros nor the names of its
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

#include <iostream>
#include <ros/shared_memory_publisher.h>
#include <ros/shared_memory_subscriber.h>
#include <test_matlab/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

ros::SharedMemoryPublisher * pub_ = NULL;

namespace ros_adapters
{

template<class __ToMessage>
struct converter{};

template<>
struct converter<test_matlab::Vector3>
{
	typedef test_matlab::Vector3 _ToMsg;

	static _ToMsg convert( const geometry_msgs::Vector3 & msg )
	{
		_ToMsg result;
		result.x = msg.x;
		result.y = msg.y;
		result.z = msg.z;
		return result;
	}
};

template<>
struct converter<geometry_msgs::Vector3>
{
	typedef geometry_msgs::Vector3 _ToMsg;

	static _ToMsg convert( const test_matlab::Vector3 & msg )
	{
		_ToMsg result;
		result.x = msg.x;
		result.y = msg.y;
		result.z = msg.z;
		return result;
	}
};

} // ros_adapters

void vec3CB( const geometry_msgs::Vector3::ConstPtr & msg )
{
	if( pub_ ) pub_->publish( ros_adapters::converter<test_matlab::Vector3>::convert( *msg ) );

	ros::shutdown();
}

int main( int argc, char ** argv )
{
	if( argc >= 2 )
	{
		//usleep( 1000*1000 );

		pub_ = new ros::SharedMemoryPublisher( argv[1] );

		ros::init( argc, argv, "test_ros" );
		ros::NodeHandle nh_rel( "~" );
		ros::Subscriber vec3_sub = nh_rel.subscribe( "vec3", 1, &vec3CB );

		ros::spin();
	}

	return 0;
}
