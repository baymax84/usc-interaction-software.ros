/***************************************************************************
 *  include/humanoid_recognizers/humanoid_recognizer_policy.h
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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>
#include <quickdev/subscriber_policy.h>
#include <quickdev/publisher_policy.h>
#include <quickdev/threading.h>

#include <humanoid/humanoid.h>

#include <visualization_msgs/MarkerArray.h>

QUICKDEV_DECLARE_POLICY_NS( HumanoidRecognizer )
{
	typedef quickdev::NodeHandlePolicy _NodeHandlePolicy;
	typedef quickdev::MessageCallbackPolicy<humanoid::_HumanoidStateArrayMsg> _HumanoidStateArrayMessageCallbackPolicy;
	typedef quickdev::PublisherPolicy<> _PublisherPolicy;
	typedef quickdev::SubscriberPolicy<> _SubscriberPolicy;
}

QUICKDEV_DECLARE_POLICY( HumanoidRecognizer, _NodeHandlePolicy, _HumanoidStateArrayMessageCallbackPolicy, _PublisherPolicy, _SubscriberPolicy )
QUICKDEV_DECLARE_POLICY_CLASS( HumanoidRecognizer )
{
	QUICKDEV_MAKE_POLICY_FUNCS( HumanoidRecognizer )

public:
	typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
	typedef visualization_msgs::Marker _MarkerMsg;
	typedef humanoid::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;

protected:

	quickdev::MessageCache<_HumanoidStateArrayMsg> states_cache_;

	QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( HumanoidRecognizer ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

	void postInit()
	{
		QUICKDEV_GET_NODEHANDLE( nh_rel );

		getMultiPub().addPublishers<_MarkerArrayMsg>( nh_rel, { "/visualization_marker_array" } );
		getMultiSub().addSubscriber( nh_rel, "humanoid_states", &HumanoidRecognizerPolicy::humanoidStatesCB, this );
	}

	QUICKDEV_ENABLE_INIT
	{
		postInit();

		QUICKDEV_SET_INITIALIZED();
	}

	void update( _MarkerArrayMsg markers )
	{
		QUICKDEV_ASSERT_INITIALIZED();

		getMultiPub().publish( "/visualization_marker_array", quickdev::make_const_shared( markers ) );
	}

	QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
	{
		auto lock = states_cache_.tryLockAndUpdate( msg );
		QUICKDEV_TRY_LOCK_OR_WARN( lock, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );

		QUICKDEV_GET_POLICY_NS( HumanoidRecognizer )::_HumanoidStateArrayMessageCallbackPolicy::invokeCallback( msg );
	}
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERPOLICY_H_
