/*******************************************************************************
 *
 *      test_pid
 * 
 *      Copyright (c) 2011, edward
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <gaze_chain/pid.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <interaction_common/math.h>

tf::TransformBroadcaster * tb;

void publishTfFrame( const tf::Transform & transform, const std::string & frame1, const std::string & frame2 )
{
	tb->sendTransform( tf::StampedTransform( transform, ros::Time::now(), frame1, frame2 ) );
}

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "test_pid" );
	ros::NodeHandle nh;

	tb = new tf::TransformBroadcaster;

	typedef double _DataType;
	typedef PidBase<_DataType> _Pid;

	_Pid pid( 30, 0, 15, -20, 20 );

	_DataType target_value = 300;

	_DataType process_value = 0, velocity = 0, acceleration = 0;

	_DataType frequency = 50;

	while ( ros::ok() )
	{
		//f = ma
		//f/m = dv/dt
		// dv = dt*f/m
		acceleration = pid.update( target_value - process_value );
		velocity += acceleration / frequency;
		process_value += 0.5 * acceleration / ( frequency * frequency ) + velocity / frequency;

		printf( "a: %f v: %f s: %f\n", acceleration, velocity, process_value );

		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( process_value ), 0, 0 ) ), "origin", "current" );
		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( target_value ), 0, 0 ) ), "origin", "target" );

		ros::Rate( frequency ).sleep();
	}

	return 0;
}
