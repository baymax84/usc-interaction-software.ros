/***************************************************************************
 *  include/quickdev_examples/action_server_node.h
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

#ifndef QUICKDEVEXAMPLES_ACTIONSERVERNODE_H_
#define QUICKDEVEXAMPLES_ACTIONSERVERNODE_H_

#include <quickdev/node.h>

#include <quickdev/action_server_policy.h>

#include <quickdev_examples/TestAction.h>

typedef quickdev_examples::TestAction _TestAction;
typedef quickdev::ActionServerPolicy<_TestAction> _TestActionServerPolicy;

QUICKDEV_DECLARE_NODE( ActionServer, _TestActionServerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ActionServer )
{
    bool enable_preempt_;

    long int count_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ActionServer ),
        count_( 0 )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        _TestActionServerPolicy::registerExecuteCB( quickdev::auto_bind( &ActionServerNode::testActionExecuteCB, this ) );
        _TestActionServerPolicy::registerPreemptCB( quickdev::auto_bind( &ActionServerNode::testActionPreemptCB, this ) );
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        enable_preempt_ = quickdev::ParamReader::readParam<bool>( nh_rel, "enable_preempt", true );
    }

    QUICKDEV_SPIN_ONCE()
    {
        PRINT_INFO( "main loop spinning!" );

        if( !_TestActionServerPolicy::active() ) return;

        count_ ++;

        _TestActionServerPolicy::_FeedbackMsg feedback_msg;
        feedback_msg.feedback = count_;
        _TestActionServerPolicy::sendFeedback( feedback_msg );
    }

    QUICKDEV_DECLARE_ACTION_PREEMPT_CALLBACK( testActionPreemptCB, _TestAction )
    {
        PRINT_INFO( "got preempt callback" );
        if( enable_preempt_ )
        {
            PRINT_INFO( "preempting current goal" );
            _TestActionServerPolicy::_ResultMsg result_msg;
            result_msg.result = count_;
            _TestActionServerPolicy::preemptAction( result_msg );
        }
        else PRINT_INFO( "ignoring preempt request" );
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( testActionExecuteCB, _TestAction )
    {
        count_ = 0;

        _TestActionServerPolicy::_FeedbackMsg feedback_msg;
        feedback_msg.feedback = count_;
        _TestActionServerPolicy::sendFeedback( feedback_msg );

        PRINT_INFO( "blocking for ten seconds" );

        if( !_TestActionServerPolicy::waitOnAction( 10 ) ) return;

        PRINT_INFO( "done; sending result" );

        _TestActionServerPolicy::_ResultMsg result_msg;
        result_msg.result = count_;
        _TestActionServerPolicy::setSuccessful( result_msg );
    }
};

#endif // QUICKDEVEXAMPLES_ACTIONSERVERNODE_H_
