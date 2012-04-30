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
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ActionServer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        _TestActionServerPolicy::registerExecuteCB( quickdev::auto_bind( &ActionServerNode::testActionExecuteCB, this ) );
        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        PRINT_INFO( "main loop spinning!" );
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( testActionExecuteCB, _TestAction )
    {
        _TestActionServerPolicy::_FeedbackMsg feedback_msg;
        _TestActionServerPolicy::sendFeedback( feedback_msg );

        PRINT_INFO( "blocking for ten seconds" );
        sleep( 10 );
        PRINT_INFO( "done; sending result" );

        _TestActionServerPolicy::_ResultMsg result_msg;
        //action_server->setSucceeded( result_msg );
        _TestActionServerPolicy::setSuccessful( result_msg );
    }
};

#endif // QUICKDEVEXAMPLES_ACTIONSERVERNODE_H_
