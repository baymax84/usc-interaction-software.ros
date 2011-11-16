/***************************************************************************
 *  include/humanoid/kinect_adapter.h
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

#ifndef HUMANOIDMODELS_HUMANOID_KINECTADAPTER_H_
#define HUMANOIDMODELS_HUMANOID_KINECTADAPTER_H_

#include <quickdev/node.h>
#include <quickdev/tf_tranceiver_policy.h>
#include <quickdev/timed_policy.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/threading.h>

#include <humanoid/humanoid.h>
#include <openni_multitracker/UserStateArray.h>

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef quickdev::TimedPolicy<> _UserStatesCBTimer;
QUICKDEV_DECLARE_NODE( KinectAdapter, _TfTranceiverPolicy, _UserStatesCBTimer )

using humanoid::_HumanoidStateArrayMsg;
using humanoid::_HumanoidStateMsg;
using humanoid::_HumanoidJointMsg;

QUICKDEV_DECLARE_NODE_CLASS( KinectAdapter )
{
private:
	typedef openni_multitracker::UserStateArray _UserStateArrayMsg;

	ros::MultiPublisher<> multi_pub_;
	ros::MultiSubscriber<> multi_sub_;

	quickdev::MessageCache<_UserStateArrayMsg> user_states_cache_;

	std::string camera_frame_;
	ros::Duration kinect_timeout_duration_;

	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( KinectAdapter ),
		camera_frame_( "/openni_depth_tracking_frame" ),
		kinect_timeout_duration_( 1.0 )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		initAll();

		QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
		multi_pub_.addPublishers<_HumanoidStateArrayMsg>( nh_rel, { "humanoid_states" } );
		multi_sub_.addSubscriber( nh_rel, "user_states", &KinectAdapterNode::userStatesCB, this );
	}

	QUICKDEV_SPIN_ONCE
	{
		if( ros::Time::now() - _UserStatesCBTimer::now() > kinect_timeout_duration_ ) return;

		auto lock = user_states_cache_.lock();
		const auto & user_states_cache = user_states_cache_.cache_;
		if( !user_states_cache ) return;

		_HumanoidStateArrayMsg state_array_msg;

		for( auto user_state = user_states_cache->user_states.begin(); user_state != user_states_cache->user_states.end(); ++user_state )
		{
			if( !user_state->is_tracked ) continue;

			_HumanoidStateMsg state_msg;
			state_msg.name = user_state->name;

			for( auto joint = humanoid::JOINT_NAMES_.begin(); joint != humanoid::JOINT_NAMES_.end(); ++joint )
			{
				const auto current_joint_frame( "/" + user_state->name + "/" + *joint );
				if( transformExists( camera_frame_, current_joint_frame ) )
				{
					const auto transform( lookupTransform( camera_frame_, current_joint_frame ) );
					_HumanoidJointMsg joint_msg;

					joint_msg.name = *joint;

					joint_msg.pose.pose.position.x = transform.getOrigin().getX();
					joint_msg.pose.pose.position.y = transform.getOrigin().getY();
					joint_msg.pose.pose.position.z = transform.getOrigin().getZ();

					joint_msg.pose.pose.orientation.x = transform.getRotation().getX();
					joint_msg.pose.pose.orientation.y = transform.getRotation().getY();
					joint_msg.pose.pose.orientation.z = transform.getRotation().getZ();
					joint_msg.pose.pose.orientation.w = transform.getRotation().getW();

					joint_msg.pose.confidence = 1.0;

					state_msg.joints.push_back( joint_msg );
				}
			}

			state_array_msg.states.push_back( state_msg );
		}

		multi_pub_.publish( "humanoid_states", quickdev::make_const_shared( state_array_msg ) );
	}

	QUICKDEV_DECLARE_MESSAGE_CALLBACK( userStatesCB, _UserStateArrayMsg )
	{
		_UserStatesCBTimer::update();

		auto lock = user_states_cache_.tryLockAndUpdate( msg );
		QUICKDEV_TRY_LOCK_OR_WARN( lock, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );
	}
};

#endif // HUMANOIDMODELS_HUMANOID_KINECTADAPTER_H_
