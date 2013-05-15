/***************************************************************************
 *  include/quickdevtests/tf_tranceiver_policy.h
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

#ifndef QUICKDEV_QUICKDEVTESTS_TFTRANCEIVERPOLICY_H_
#define QUICKDEV_QUICKDEVTESTS_TFTRANCEIVERPOLICY_H_

#include <quickdev/node.h>
#include <quickdev/tf_tranceiver_policy.h>

QUICKDEV_DECLARE_NODE( TfTranceiverPolicy, quickdev::TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( TfTranceiverPolicy )
{
public:
	ros::Time last_time_;
	ros::Time now_;

	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( TfTranceiverPolicy ),
		last_time_( ros::Time::now() ), now_( ros::Time::now() )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		tf::Transform transform( tf::Quaternion( 0, 0, 0, 1 ), tf::Vector3( 0, 0, 0 ) );
		publishTransform( transform, "/world", "/frame1", now_ );
		publishTransform( transform, "/world", "/frame2", now_ );

		lookupTransform( "/world", "/frame1" );
		lookupTransform( "/world", "/frame2" );

		lookupTransform( "/world", "/frame1", now_ - ros::Duration( 5 ) );
		lookupTransform( "/world", "/frame2", now_ - ros::Duration( 5 ) );
	}

	QUICKDEV_SPIN_ONCE
	{
		now_ = ros::Time::now();

		auto frame1_last_to_frame2_past( lookupTransform( "/frame1", last_time_, "/frame2", last_time_ - ros::Duration( 1 ), "/world" ) );
		frame1_last_to_frame2_past.child_frame_id_ = "/frame2_past";

		publishTransform( frame1_last_to_frame2_past, now_ );

		// get the last state of /world -> /frame1
		auto world_to_frame1_last( lookupTransform( "/world", "/frame1", last_time_ ) );
		// get the last state of /world -> /frame1
		auto world_to_frame2_last( lookupTransform( "/world", "/frame2", last_time_ ) );

		world_to_frame1_last.getOrigin().setX( world_to_frame1_last.getOrigin().x() +  1 * ( now_ - last_time_ ).toSec() );
		world_to_frame2_last.getOrigin().setY( world_to_frame2_last.getOrigin().y() +  1 * ( now_ - last_time_ ).toSec() );

		// update /world -> frame1 at the current time
		publishTransform( world_to_frame1_last, now_ );
		// update /world -> frame2 at the current time
		publishTransform( world_to_frame2_last, now_ );

		last_time_ = now_;
	}
};

#endif // QUICKDEV_QUICKDEVTESTS_TFTRANCEIVERPOLICY_H_
