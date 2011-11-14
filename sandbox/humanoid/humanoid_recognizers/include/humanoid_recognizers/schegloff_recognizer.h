/***************************************************************************
 *  include/humanoid_recognizers/schegloff_recognizer.h
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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_SCHEGLOFFRECOGNIZER_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_SCHEGLOFFRECOGNIZER_H_

#include <quickdev/node.h>

#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

#include <humanoid/humanoid.h>

#include <visualization_msgs/MarkerArray.h>

QUICKDEV_DECLARE_NODE( SchegloffRecognizer )

using humanoid::_HumanoidStateArrayMsg;

QUICKDEV_DECLARE_NODE_CLASS( SchegloffRecognizer )
{
private:
	typedef visualization_msgs::Marker _MarkerArrayMsg;

	ros::MultiSubscriber<> multi_sub_;
	ros::MultiPublisher<> multi_pub_;

	_HumanoidStateArrayMsg::ConstPtr states_cache_;

	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( SchegloffRecognizer )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		initAll();

		QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

		multi_pub_.addPublishers<_MarkerArrayMsg>( nh_rel, { "/visualization_marker_array" } );
		multi_sub_.addSubscriber( nh_rel, "humanoid_states", &SchegloffRecognizerNode::humanoidStatesCB, this );
	}

	QUICKDEV_SPIN_ONCE
	{
		if( !states_cache_ ) return;

		_MarkerArrayMsg markers;

		for( auto humanoid = states_cache_->states.begin(); humanoid != states_cache_->states.end(); ++humanoid )
		{
			//
		}

		multi_pub_.publish( "/visualization_marker_array", quickdev::make_const_shared( markers ) );
	}

	QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
	{
		states_cache_ = msg;
	}
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_SCHEGLOFFRECOGNIZER_H_
