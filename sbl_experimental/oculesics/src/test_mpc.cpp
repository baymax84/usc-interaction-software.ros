/*******************************************************************************
 *
 *      test_mpc
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

#include <gaze_chain/mpc.h>
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

	const unsigned int num_signals = 3;

	// strain has least priority since it can be overridden by the need to look at an interesting point
	const unsigned int STRAIN = 2;
	const unsigned int SALIENCY = 1;
	// the social signal has the highest priority; given a social situation, all other signals can be overridden
	const unsigned int SOCIAL = 0;

	typedef double _DataType;
	typedef MultiplexedController<_DataType, num_signals, MultiplexedControllerParams::OutputMode::SIGNAL> _Controller;
	typedef _Controller::_Settings _Settings;

	typedef PidBase<_DataType> _Pid;

	_Pid pid( 30, 0, 12, -20, 20 );

	_Controller::_InputVector signals( num_signals );

	std::vector<_Settings> controller_settings( num_signals );

	controller_settings[STRAIN] = _Settings( -1000, -20, 20, 1000 );
	controller_settings[SALIENCY] = _Settings( -300, -20, 20, 300 );
	controller_settings[SOCIAL] = _Settings( -200, -20, 20, 200 );

	_Controller controller( controller_settings );

	_DataType social_target = -90, saliency_target = -45, strain_target = 45;

	_DataType signal_value = 0, velocity = 0, acceleration = 0, pid_target = 0;

	_DataType frequency = 50;

	while ( ros::ok() )
	{
		signals[STRAIN] = _Controller::_InputDataType( strain_target - signal_value );
		signals[SALIENCY] = _Controller::_InputDataType( saliency_target - signal_value );
		signals[SOCIAL] = _Controller::_InputDataType( social_target - signal_value );

		pid_target = signal_value - controller.update( signals );

		printf( "--------------------\n" );

		// start simulation
		acceleration = pid.update( pid_target - signal_value );
		velocity += acceleration / frequency;
		signal_value += 0.5 * acceleration / ( frequency * frequency ) + velocity / frequency;

		printf( "pid_target: %f signal_value: %f\n", pid_target, signal_value );

		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( signal_value ), 0, 0 ) ), "origin", "current" );
		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( social_target ), 0, 0 ) ), "origin", "social_target" );
		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( saliency_target ), 0, 0 ) ), "origin", "saliency_target" );
		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( strain_target ), 0, 0 ) ), "origin", "strain_target" );
		publishTfFrame( tf::Transform( tf::Quaternion( degToRad( pid_target ), 0, 0 ) ), "origin", "pid_target" );

		ros::Rate( frequency ).sleep();
		// end simulation
	}

	return 0;
}
