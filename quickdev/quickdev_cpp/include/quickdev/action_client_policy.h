/***************************************************************************
 *  include/quickdev/action_client_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_ACTIONCLIENTPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_ACTIONCLIENTPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE
{

QUICKDEV_DECLARE_POLICY( ActionClient, NodeHandlePolicy );

template<class __Action, unsigned int __Id__>
QUICKDEV_DECLARE_POLICY_CLASS( ActionClient )
{
	QUICKDEV_MAKE_POLICY_FUNCS( ActionClient )

public:
	typedef actionlib::SimpleActionClient<__Action> _ActionClient;
	typedef actionlib::SimpleClientGoalState _GoalState;
	typedef __QUICKDEV_FUNCTION_TYPE<void()> _TimeoutCallback;
	typedef typename __Action::Goal _Goal;
	typedef typename __Action::Feedback _Feedback;
	typedef typename __Action::Result _Result;

private:
	_ActionClient * action_client_;
	std::string action_topic_name_;
	_TimeoutCallback timeout_callback_;
	ros::Time timeout_timestamp_;
	bool enable_timeout_;

	QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( ActionClient ),
		action_client_( NULL ),
		enable_timeout_( false ),
		initialized_( false )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

	~ActionClientPolicy()
	{
		if( action_client_ ) delete action_client_;
	}

	void postInit()
	{
		auto nh_rel = NodeHandlePolicy::getNodeHandle();

		action_client_( new _ActionClient( nh_rel, action_topic_name_ ) );
	}

	QUICKDEV_ENABLE_INIT
	{
		postInit();
		QUICKDEV_SET_INITIALIZED();
	}

	void doneCB( const _GoalState & state, const typename _Result::ConstPtr& result )
	{
		PRINT_INFO( "Finished in state [%s]", state.toString().c_str() );

		// callback( state, result )
	}

	// Called once when the goal becomes active
	void activeCB()
	{
		PRINT_INFO( "Goal just went active" );

		// callback()
	}

	// Called every time feedback is received for the goal
	QUICKDEV_DECLARE_MESSAGE_CALLBACK2( feedbackCB, typename _Feedback, feedback )
	{
		// callback( feedback )
	}

	void registerTimeout( const double & duration, const _TimeoutCallback & callback )
	{
		registerTimeout( ros::Duration( duration ), callback );
	}

	void registerTimeout( const ros::Duration & duration, const _TimeoutCallback & callback )
	{
		QUICKDEV_ASSERT_INITIALIZED();

		if( duration.isZero() )
		{
			PRINT_WARN( "Registering a timeout with a duration of zero has no effect" );
			return;
		}

		timeout_timestamp_ = ros::Time::now() + duration;
		timeout_callback_ = callback;
		enable_timeout_ = true;
	}

	bool sendGoalAndWait( const _Goal & goal, const double & duration )
	{
		return sendGoalAndWait( goal, ros::Duration( duration ) );
	}

	bool sendGoalAndWait( const _Goal & goal, const ros::Duration & duration )
	{
		sendGoal( goal );
		return waitForResult( duration );
	}

	void sendGoalAndWait( const _Goal & goal, const double & duration, const _TimeoutCallback & callback )
	{
		sendGoalAndWait( goal, ros::Duration( duration ), callback );
	}

	void sendGoalAndWait( const _Goal & goal, const ros::Duration & duration, const _TimeoutCallback & callback )
	{
		sendGoal( goal );
		waitForResult( duration, callback );
	}

	bool waitForResult( const double & duration )
	{
		return waitForResult( ros::Duration( duration ) );
	}

	bool waitForResult( const ros::Duration & duration )
	{
		QUICKDEV_ASSERT_INITIALIZED( false );

		return action_client_->waitForResult( duration );
	}

	void waitForResult( const double & duration, const _TimeoutCallback & callback )
	{
		return waitForResult( ros::Duration( duration ), callback );
	}

	void waitForResult( const ros::Duration & duration, const _TimeoutCallback & callback )
	{
		registerTimeout( duration, callback );
	}

	_GoalState getState()
	{
		QUICKDEV_CHECK_INITIALIZED();

		return action_client_->getState();
	}

	void sendGoal( const _Goal & goal )
	{
		QUICKDEV_ASSERT_INITIALIZED();

		if( !action_client_ )
		{
			PRINT_ERROR( "Cannot send goal to un-initialized client" );
			return;
		}

		PRINT_INFO( "Waiting for action server to start." );
		action_client_->waitForServer();

		PRINT_INFO( "Action server started; sending goal." );
		// send a goal to the action
		action_client_->sendGoal( goal );
	}

	void update()
	{
		QUICKDEV_ASSERT_INITIALIZED();

		if( enable_timeout_ && !timeout_timestamp_.isZero() && ros::Time::now() > timeout_timestamp_ )
		{
			enable_timeout_ = false;
			timeout_callback_();
		}
	}
};

}

#endif // QUICKDEVCPP_QUICKDEV_ACTIONCLIENTPOLICY_H_
