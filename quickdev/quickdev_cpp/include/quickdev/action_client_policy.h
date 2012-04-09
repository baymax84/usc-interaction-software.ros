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
#include <quickdev/auto_bind.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================

template<class __Action>
QUICKDEV_DECLARE_POLICY2( ActionClient, NodeHandlePolicy, CallbackPolicy<void()>, MessageCallbackPolicy<typename __Action::_action_feedback_type::_feedback_type>, CallbackPolicy<void( actionlib::SimpleClientGoalState const &, typename __Action::_action_result_type::_result_type::ConstPtr const & )> );

template<class __Action, unsigned int __Id__ = 0>
QUICKDEV_DECLARE_POLICY_CLASS2( ActionClient, __Action )
{
    QUICKDEV_MAKE_POLICY_FUNCS( ActionClient )

public:
    typedef actionlib::SimpleActionClient<__Action> _ActionClient;
    typedef actionlib::SimpleClientGoalState _GoalState;
    typedef __QUICKDEV_FUNCTION_TYPE<void()> _TimeoutCallback;
    typedef typename __Action::_action_goal_type::_goal_type _GoalMsg;
    typedef typename __Action::_action_feedback_type::_feedback_type _FeedbackMsg;
    typedef typename __Action::_action_result_type::_result_type _ResultMsg;

    typedef CallbackPolicy<void()> _ActiveCallbackPolicy;
    typedef MessageCallbackPolicy<typename __Action::_action_feedback_type::_feedback_type> _FeedbackCallbackPolicy;
    typedef CallbackPolicy<void( actionlib::SimpleClientGoalState const &, typename __Action::_action_result_type::_result_type::ConstPtr const & )> _DoneCallbackPolicy;

private:
    _ActionClient * action_client_;
    std::string action_name_;
    std::string action_topic_name_;
    _TimeoutCallback timeout_callback_;
    ros::Time timeout_timestamp_;
    bool enable_timeout_;

    // =========================================================================================================================================

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR2( ActionClient, __Action ),
        action_client_( NULL ),
        enable_timeout_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    ~ActionClientPolicy();

    // =========================================================================================================================================

    QUICKDEV_ENABLE_INIT();

    //! Called once when the goal becomes active
    void activeCB();

    //! Called every time feedback is received for the goal
    QUICKDEV_DECLARE_MESSAGE_CALLBACK2( feedbackCB, typename _FeedbackMsg, feedback );

    //! Called once when the goal is completed
    void doneCB( _GoalState const & state, typename _ResultMsg::ConstPtr const & result );

    template<class... __Args>
    void registerActiveCB( __Args&&... args );

    template<class... __Args>
    void registerFeedbackCB( __Args&&... args );

    template<class... __Args>
    void registerDoneCB( __Args&&... args );

    void registerTimeout( double const & duration, _TimeoutCallback const & callback );

    void registerTimeout( ros::Duration const & duration, _TimeoutCallback const & callback );

    bool sendGoalAndWait( _GoalMsg const & goal, double const & duration );

    bool sendGoalAndWait( _GoalMsg const & goal, ros::Duration const & duration );

    void sendGoalAndWait( _GoalMsg const & goal, double const & duration, _TimeoutCallback const & callback );

    void sendGoalAndWait( _GoalMsg const & goal, ros::Duration const & duration, _TimeoutCallback const & callback );

    bool waitForResult( double const & duration );

    bool waitForResult( ros::Duration const & duration );

    void waitForResult( double const & duration, _TimeoutCallback const & callback );

    void waitForResult( ros::Duration const & duration, _TimeoutCallback const & callback );

    _GoalState getState();

    void sendGoal( _GoalMsg const & goal );

    void update();

    void interruptAction();
};

// #############################################################################################################################################

#include <quickdev/details/action_client_policy_impl.h>

}

#endif // QUICKDEVCPP_QUICKDEV_ACTIONCLIENTPOLICY_H_
