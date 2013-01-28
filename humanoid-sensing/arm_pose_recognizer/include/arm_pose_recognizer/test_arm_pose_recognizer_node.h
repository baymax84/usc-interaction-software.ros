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
typedef humanoid_sensing_msgs::MetaJoint _MetaJointMsg;

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


        auto & meta_joints = goal.meta_joints;
        meta_joints.resize( 10 );

        meta_joints[0].name = "user1_neck";
        meta_joints[0].parent_name = "";
        meta_joints[0].start_frame_name = "/user1/neck";
        meta_joints[0].end_frame_name = "/user1/neck";

        meta_joints[1].name = "user1_elbow_r";
        meta_joints[1].parent_name = "user1_neck";
        meta_joints[1].start_frame_name = "/user1/right_elbow";
        meta_joints[1].end_frame_name = "/user1/right_elbow";

        meta_joints[2].name = "user1_elbow_l";
        meta_joints[2].parent_name = "user1_neck";
        meta_joints[2].start_frame_name = "/user1/left_elbow";
        meta_joints[2].end_frame_name = "/user1/left_elbow";

        meta_joints[3].name = "user1_hand_r";
        meta_joints[3].parent_name = "user1_elbow_r";
        meta_joints[3].start_frame_name = "/user1/right_hand";
        meta_joints[3].end_frame_name = "/user1/right_hand";

        meta_joints[4].name = "user1_hand_l";
        meta_joints[4].parent_name = "user1_elbow_l";
        meta_joints[4].start_frame_name = "/user1/left_hand";
        meta_joints[4].end_frame_name = "/user1/left_hand";


        meta_joints[5].name = "user2_neck";
        meta_joints[5].parent_name = "";
        meta_joints[5].start_frame_name = "/user2/neck";
        meta_joints[5].end_frame_name = "/user2/neck";

        meta_joints[6].name = "user2_elbow_r";
        meta_joints[6].parent_name = "user2_neck";
        meta_joints[6].start_frame_name = "/user2/right_elbow";
        meta_joints[6].end_frame_name = "/user2/right_elbow";

        meta_joints[7].name = "user2_elbow_l";
        meta_joints[7].parent_name = "user2_neck";
        meta_joints[7].start_frame_name = "/user2/left_elbow";
        meta_joints[7].end_frame_name = "/user2/left_elbow";

        meta_joints[8].name = "user2_hand_r";
        meta_joints[8].parent_name = "user2_elbow_r";
        meta_joints[8].start_frame_name = "/user2/right_hand";
        meta_joints[8].end_frame_name = "/user2/right_hand";

        meta_joints[9].name = "user2_hand_l";
        meta_joints[9].parent_name = "user2_elbow_l";
        meta_joints[9].start_frame_name = "/user2/left_hand";
        meta_joints[9].end_frame_name = "/user2/left_hand";

        goal.desired_joint_names = { "user1_elbow_r", "user1_elbow_l", "user1_hand_r", "user1_hand_l" };
        goal.observed_joint_names = { "user2_elbow_r", "user2_elbow_l", "user2_hand_r", "user2_hand_l" };
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

        auto const & match_qualities = result->match_qualities;

        for( size_t i = 0; i < match_qualities.size(); ++i )
        {
            auto const & match_quality = match_qualities[i];
            PRINT_INFO( "joint[%zu] match quality: ( %f, %f, %f )", i, match_quality.x, match_quality.y, match_quality.z );
        }

        QUICKDEV_GET_RUNABLE_POLICY()::interrupt();
    }
};

#endif // ARMPOSERECOGNIZER_TESTARMPOSERECOGNIZERNODE_H_
