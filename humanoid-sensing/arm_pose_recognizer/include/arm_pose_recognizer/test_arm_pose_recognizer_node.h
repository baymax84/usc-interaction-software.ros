/***************************************************************************
 *  include/arm_pose_recognizer/test_arm_pose_recognizer_node.h
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
 *  * Neither the name of interaction-ros-pkg nor the names of its
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

#ifndef ARMPOSERECOGNIZER_TESTARMPOSERECOGNIZERNODE_H_
#define ARMPOSERECOGNIZER_TESTARMPOSERECOGNIZERNODE_H_

#include <quickdev/node.h>

#include <quickdev/action_client_policy.h>
//#include <quickdev/tf_tranceiver_policy.h>

#include <arm_pose_recognizer/EvaluatePoseAction.h>

typedef arm_pose_recognizer::EvaluatePoseAction _EvaluatePoseAction;
typedef quickdev::ActionClientPolicy<_EvaluatePoseAction> _EvaluatePoseActionClientPolicy;

QUICKDEV_DECLARE_NODE( TestArmPoseRecognizer, _EvaluatePoseActionClientPolicy )

QUICKDEV_DECLARE_NODE_CLASS( TestArmPoseRecognizer )
{
    quickdev::ActionToken<_EvaluatePoseActionClientPolicy> action_token_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( TestArmPoseRecognizer )
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
        _EvaluatePoseActionClientPolicy::registerActiveCB( quickdev::auto_bind( &TestArmPoseRecognizerNode::evaluatePoseActionActiveCB, this ) );
        _EvaluatePoseActionClientPolicy::registerFeedbackCB( quickdev::auto_bind( &TestArmPoseRecognizerNode::evaluatePoseActionFeedbackCB, this ) );
        _EvaluatePoseActionClientPolicy::registerDoneCB( quickdev::auto_bind( &TestArmPoseRecognizerNode::evaluatePoseActionDoneCB, this ) );

        initPolicies<_EvaluatePoseActionClientPolicy>( "action_name_param", std::string( "evaluate_pose" ) );

        initPolicies<quickdev::policy::ALL>();

        _EvaluatePoseActionClientPolicy::_GoalMsg goal;

        goal.desired_pose_frame_names = { "/user1/neck", "/user1/left_elbow", "/user1/right_elbow", "/user1/left_hand", "/user1/right_hand" };
        goal.observed_pose_frame_names = { "/user2/neck", "/user2/left_elbow", "/user2/right_elbow", "/user2/left_hand", "/user2/right_hand" };
        goal.variance.x = 0.2;
        goal.variance.y = 0.2;
        goal.variance.z = 0.2;

        auto token = _EvaluatePoseActionClientPolicy::sendGoal( goal );
        action_token_ = token;
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }

    QUICKDEV_DECLARE_ACTION_ACTIVE_CALLBACK( evaluatePoseActionActiveCB )
    {
        PRINT_INFO( "client detected goal started" );
    }

    QUICKDEV_DECLARE_ACTION_FEEDBACK_CALLBACK( evaluatePoseActionFeedbackCB, _EvaluatePoseAction )
    {
        PRINT_INFO( "client got feedback" );
    }

    QUICKDEV_DECLARE_ACTION_DONE_CALLBACK( evaluatePoseActionDoneCB, _EvaluatePoseAction )
    {
        PRINT_INFO( "client got goal result" );

        PRINT_INFO( "l_hand x: %f\n", result->l_hand_match_quality.x );
        PRINT_INFO( "l_hand y: %f\n", result->l_hand_match_quality.y );
        PRINT_INFO( "l_hand z: %f\n", result->l_hand_match_quality.z );

        PRINT_INFO( "r_hand x: %f\n", result->r_hand_match_quality.x );
        PRINT_INFO( "r_hand y: %f\n", result->r_hand_match_quality.y );
        PRINT_INFO( "r_hand z: %f\n", result->r_hand_match_quality.z );

        PRINT_INFO( "l_elbow x: %f\n", result->l_elbow_match_quality.x );
        PRINT_INFO( "l_elbow y: %f\n", result->l_elbow_match_quality.y );
        PRINT_INFO( "l_elbow z: %f\n", result->l_elbow_match_quality.z );

        PRINT_INFO( "r_elbow x: %f\n", result->r_elbow_match_quality.x );
        PRINT_INFO( "r_elbow y: %f\n", result->r_elbow_match_quality.y );
        PRINT_INFO( "r_elbow z: %f\n", result->r_elbow_match_quality.z );

        QUICKDEV_GET_RUNABLE_POLICY()::interrupt();
    }
};

#endif // ARMPOSERECOGNIZER_TESTARMPOSERECOGNIZERNODE_H_
