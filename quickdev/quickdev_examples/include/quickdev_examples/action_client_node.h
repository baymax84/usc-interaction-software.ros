/***************************************************************************
 *  include/quickdev_examples/action_client_node.h
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

#ifndef QUICKDEVEXAMPLES_ACTIONCLIENTNODE_H_
#define QUICKDEVEXAMPLES_ACTIONCLIENTNODE_H_

#include <quickdev/node.h>
#include <quickdev/action_client_policy.h>

#include <quickdev_examples/TestAction.h>

typedef quickdev_examples::TestAction _TestAction;
typedef quickdev::ActionClientPolicy<_TestAction> _TestActionClientPolicy;

QUICKDEV_DECLARE_NODE( ActionClient, _TestActionClientPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ActionClient )
{
    quickdev::ActionToken<_TestActionClientPolicy> action_token_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ActionClient )
    {
        //
    }

    void setup()
    {
        // update ROS in a separate thread so we're free to block within spinOnce()
        initPolicies<quickdev::RunablePolicy>( "spin_ros_thread_param", true );
    }

    QUICKDEV_SPIN_FIRST()
    {
        _TestActionClientPolicy::registerActiveCB( quickdev::auto_bind( &ActionClientNode::testActionActiveCB, this ) );
        _TestActionClientPolicy::registerFeedbackCB( quickdev::auto_bind( &ActionClientNode::testActionFeedbackCB, this ) );
        _TestActionClientPolicy::registerDoneCB( quickdev::auto_bind( &ActionClientNode::testActionDoneCB, this ) );

        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        if( !quickdev::ParamReader::readParam<bool>( nh_rel, "enable_goal", true ) ) return;

        _TestActionClientPolicy::_GoalMsg goal;

        // test
        auto token = _TestActionClientPolicy::sendGoal( goal );
        action_token_ = token;
    }

    QUICKDEV_SPIN_ONCE()
    {
        PRINT_INFO( "spinning!" );
        PRINT_INFO( "server state: %s", _TestActionClientPolicy::getState().toString().c_str() );

        ros::Duration( 5 ).sleep();

        action_token_.cancel();
    }

    QUICKDEV_DECLARE_ACTION_ACTIVE_CALLBACK( testActionActiveCB )
    {
        PRINT_INFO( "client detected goal started" );
    }

    QUICKDEV_DECLARE_ACTION_FEEDBACK_CALLBACK( testActionFeedbackCB, _TestAction )
    {
        PRINT_INFO( "client got feedback" );
    }

    QUICKDEV_DECLARE_ACTION_DONE_CALLBACK( testActionDoneCB, _TestAction )
    {
        PRINT_INFO( "client got goal result" );

        QUICKDEV_GET_RUNABLE_POLICY()::interrupt();
    }

};

#endif // QUICKDEVEXAMPLES_ACTIONCLIENTNODE_H_
