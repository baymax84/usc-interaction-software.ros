/***************************************************************************
 *  include/quickdev/action_server_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_ACTIONSERVERPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_ACTIONSERVERPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/callback_policy.h>

#include <quickdev/auto_bind.h>
#include <quickdev/time.h>

#include <actionlib/server/simple_action_server.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
template<class __Action, unsigned int __Id__ = 0>
class ActionServerPolicy
:
    public GenericPolicyAdapter
    <
        NodeHandlePolicy,
        // executeCB( goal, server )
        CallbackPolicy<void( typename __Action::_action_goal_type::_goal_type::ConstPtr const &, actionlib::SimpleActionServer<__Action> * const )>
    >
{
    QUICKDEV_MAKE_POLICY_FUNCS( ActionServer )

public:
    typedef type_utils::_TimedMutex _Mutex;

    typedef actionlib::SimpleActionServer<__Action> _ActionServer;

    typedef typename __Action::_action_goal_type::_goal_type _GoalMsg;
    typedef typename __Action::_action_feedback_type::_feedback_type _FeedbackMsg;
    typedef typename __Action::_action_result_type::_result_type _ResultMsg;

    typedef __QUICKDEV_FUNCTION_TYPE<void ( typename _GoalMsg::ConstPtr const &, _ActionServer * const )> _ExecuteCallback;
    typedef __QUICKDEV_FUNCTION_TYPE<void ( _ActionServer * const )> _PreemptCallback;
    typedef __QUICKDEV_FUNCTION_TYPE<void ( _ActionServer * const )> _GoalCallback;

    typedef typename CallbackPolicy_types::from_function<_ExecuteCallback>::type _ExecuteCallbackPolicy;

    typedef GenericPolicyAdapter<NodeHandlePolicy, _ExecuteCallbackPolicy> _PolicyAdapter;

private:
    _ActionServer * action_server_;

    bool preempt_accepted_;

    _PreemptCallback preempt_callback_;
    _GoalCallback goal_callback_;

    _Mutex action_mutex_;

    _GoalMsg goal_;

    _ResultMsg result_;

    std::string action_topic_name_;

    // =========================================================================================================================================

public:
    template<class... __Args>
    ActionServerPolicy( __Args&&... args )
    :
        _PolicyAdapter( std::forward<__Args>( args )... ),
        action_server_( NULL ),
        preempt_accepted_( false ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    // =========================================================================================================================================

    QUICKDEV_ENABLE_INIT();

    QUICKDEV_DECLARE_CONST_ACCESSOR( goal_, Goal );

    //! Callback for execute requests from action server
    QUICKDEV_DECLARE_MESSAGE_CALLBACK( executeActionCB, typename _GoalMsg );

    //! Callback for preempt requests from action server
    void preemptCB();

    //! Callback for goal requests from action server
    void goalCB();

    //! Allow others to register an execute callback
    template<class... __Args>
    void registerExecuteCB( __Args&&... args );

    //! Allow others to register a preempt callback
    void registerPreemptCB( _PreemptCallback const & callback );

    //! Allow others to register a goal callback
    void registerGoalCB( _GoalCallback const & callback );

    //! Publish feedback to client
    template<class... __Args>
    void sendFeedback( __Args&&... args );

    //! Set state of action to successful
    template<class... __Args>
    void setSuccessful( __Args&&... args );

    //! Set state of action to aborted
    template<class... __Args>
    void setAborted( __Args&&... args );

    //! Set state of action to preempted
    template<class... __Args>
    void setPreempted( __Args&&... args );

    //! When executing an action, halt the action and indicate that the action has been completed
    template<class... __Args>
    void completeAction( __Args&&... args );

    //! When executing an action, halt the action and indicate that the action has been aborted
    template<class... __Args>
    void abortAction( __Args&&... args );

    //! When executing an action, halt the action and indicate that the action has been preempted
    template<class... __Args>
    void preemptAction( __Args&&... args );

    //! Lock the current thread (should be the execute thread) until the action has completed (or aborted, or preempted)
    bool waitOnAction( double const & wait_time = 0 );

    //! Whether we've received a request to preempt the current action
    bool preemptRequested();

    //! Whether we've received a request to preempt the current action and accepted that request
    bool const & preemptAccepted() const;

    //! Whether there's an active goal
    bool active();
};

// #############################################################################################################################################

#include <quickdev/details/action_server_policy_impl.h>

}

#endif // QUICKDEVCPP_QUICKDEV_ACTIONSERVERPOLICY_H_
