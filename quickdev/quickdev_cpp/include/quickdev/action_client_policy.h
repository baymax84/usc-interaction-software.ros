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

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
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

    // =========================================================================================================================================

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( ActionClient ),
        action_client_( NULL ),
        enable_timeout_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    // =========================================================================================================================================

    QUICKDEV_ENABLE_INIT()
    {
        postInit();
        QUICKDEV_SET_INITIALIZED();
    }

    //! Called every time feedback is received for the goal
    QUICKDEV_DECLARE_MESSAGE_CALLBACK2( feedbackCB, typename _Feedback, feedback );
    // =========================================================================================================================================

    ~ActionClientPolicy();

    void postInit();

    void doneCB( _GoalState const & state, typename _Result::ConstPtr const & result );

    //! Called once when the goal becomes active
    void activeCB();

    void registerTimeout( double const & duration, _TimeoutCallback const & callback );

    void registerTimeout( ros::Duration const & duration, _TimeoutCallback const & callback );

    bool sendGoalAndWait( _Goal const & goal, double const & duration );

    bool sendGoalAndWait( _Goal const & goal, ros::Duration const & duration );

    void sendGoalAndWait( _Goal const & goal, double const & duration, _TimeoutCallback const & callback );

    void sendGoalAndWait( _Goal const & goal, ros::Duration const & duration, _TimeoutCallback const & callback );

    bool waitForResult( double const & duration );

    bool waitForResult( ros::Duration const & duration );

    void waitForResult( double const & duration, _TimeoutCallback const & callback );

    void waitForResult( ros::Duration const & duration, _TimeoutCallback const & callback );

    _GoalState getState();

    void sendGoal( _Goal const & goal );

    void update();
};

// #############################################################################################################################################

#include <quickdev/details/action_client_policy_impl.h>

}

#endif // QUICKDEVCPP_QUICKDEV_ACTIONCLIENTPOLICY_H_
