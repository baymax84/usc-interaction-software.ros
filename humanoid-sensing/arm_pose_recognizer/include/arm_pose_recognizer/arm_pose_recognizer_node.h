/***************************************************************************
 *  include/arm_pose_recognizer/arm_pose_recognizer_node.h
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

#ifndef ARMPOSERECOGNIZER_ARMPOSERECOGNIZERNODE_H_
#define ARMPOSERECOGNIZER_ARMPOSERECOGNIZERNODE_H_

#include <quickdev/node.h>

#include <quickdev/action_server_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

#include <arm_pose_recognizer/pose.h>

#include <arm_pose_recognizer/EvaluatePoseAction.h>

typedef arm_pose_recognizer::EvaluatePoseAction _EvaluatePoseAction;
typedef quickdev::ActionServerPolicy<_EvaluatePoseAction> _EvaluatePoseActionServerPolicy;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

// Declare a node called ArmPoseRecognizerNode.
// A quickdev::RunablePolicy is automatically prepended to the list of policies our node will use.
// To use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( ArmPoseRecognizer, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( ArmPoseRecognizer, _EvaluatePoseActionServerPolicy, _TfTranceiverPolicy )

// Declare a class called ArmPoseRecognizerNode
//
QUICKDEV_DECLARE_NODE_CLASS( ArmPoseRecognizer )
{
    typedef Pose<btVector3> _Pose;

    // Variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ArmPoseRecognizer ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ArmPoseRecognizer )
    {
        //
    }

    // This function is called by quickdev::RunablePolicy after all policies are constructed but just before the main loop is started.
    // All policy initialization should be done here.
    //
    QUICKDEV_SPIN_FIRST()
    {
        // Say we had a policy called _SomePolicy that looked for the meta-parameter "some_value1_param" of type SomeType and
        // "some_value2_param" of type SomeOtherType in its init function
        // We can create those meta-params here and then pass them to all policies using initPolicies<...>():
        //
        // initPolicies<quickdev::policy::ALL>( "some_value1_param", SomeType(), "some_value2_param", SomeOtherType() );
        //
        // Or we can pass those meta-params only to _SomePolicy by specifying its type:
        //
        // initPolicies<_SomePolicy>( "some_value1_param", SomeType(), "some_value2_param", SomeOtherType() );
        //
        // If we want to initialize all policies and use their default values, we can simply call initPolicies<quickdev::policy::ALL>()
        // with no arguments.
        // Note that most initable policies won't function properly unless their init() functions are called directly or via initPolicies<...>().
        // Furthermore, since each policy is required to track its initialization state, initPolicies<...>() is guaranteed to only call init()
        // on policies that have yet to be initialized; therefore, calling initPolicies<quickdev::policy::ALL>() at the end of QUICKDEV_SPIN_FIRST()
        // is always a safe operation.
        // To instead force re-initialization, call forceInitPolicies<...>().
        //

        _EvaluatePoseActionServerPolicy::registerExecuteCB( quickdev::auto_bind( &ArmPoseRecognizerNode::evaluatePoseActionExecuteCB, this ) );

        initPolicies<_EvaluatePoseActionServerPolicy>( "action_name_param", std::string( "evaluate_pose" ) );
        initPolicies<quickdev::policy::ALL>();
    }

    // This optional function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate).
    // Most updateable policies should have their update( ... ) functions called within this context.
    //
    QUICKDEV_SPIN_ONCE()
    {
        if( !_EvaluatePoseActionServerPolicy::active() ) return;

        // look up tf frames

        auto const & goal = _EvaluatePoseActionServerPolicy::getGoal();

        auto const & variance = goal.variance;
        auto const & desired_frame_names = goal.desired_pose_frame_names;
        auto const & observed_frame_names = goal.observed_pose_frame_names;

        if( desired_frame_names.size() != observed_frame_names.size() )
        {
            PRINT_ERROR( "The number of desired frame names must be the same as the number of observed frame names; aborting action." );
            return abortAction();
        }

        auto desired_frame_name_it = desired_frame_names.cbegin() + 1;
        auto observed_frame_name_it = observed_frame_names.cbegin() + 1;

        _Pose desired_pose;
        _Pose observed_pose;

        auto desired_pose_it = desired_pose.begin();
        auto observed_pose_it = observed_pose.begin();

        for( ; desired_frame_name_it != desired_frame_names.cend(); ++desired_frame_name_it, ++observed_frame_name_it, ++desired_pose_it, ++observed_pose_it )
        {
            auto const desired_tf = _TfTranceiverPolicy::lookupTransform( *desired_frame_names.cbegin(), *desired_frame_name_it );
            auto const observed_tf = _TfTranceiverPolicy::lookupTransform( *observed_frame_names.cbegin(), *observed_frame_name_it );

            *desired_pose_it = desired_tf.getOrigin();
            *observed_pose_it = observed_tf.getOrigin();
        }

        desired_pose.normalize();
        observed_pose.normalize();

        // perform distance calculation
        auto pose_error = observed_pose - desired_pose;

        _EvaluatePoseActionServerPolicy::_ResultMsg result_msg;
        result_msg.l_hand_match_quality.x = pose_error.l_hand.getX() / variance.x;
        result_msg.l_hand_match_quality.y = pose_error.l_hand.getY() / variance.y;
        result_msg.l_hand_match_quality.z = pose_error.l_hand.getZ() / variance.z;

        result_msg.r_hand_match_quality.x = pose_error.r_hand.getX() / variance.x;
        result_msg.r_hand_match_quality.y = pose_error.r_hand.getY() / variance.y;
        result_msg.r_hand_match_quality.z = pose_error.r_hand.getZ() / variance.z;

        result_msg.l_elbow_match_quality.x = pose_error.l_elbow.getX() / variance.x;
        result_msg.l_elbow_match_quality.y = pose_error.l_elbow.getY() / variance.y;
        result_msg.l_elbow_match_quality.z = pose_error.l_elbow.getZ() / variance.z;

        result_msg.r_elbow_match_quality.x = pose_error.r_elbow.getX() / variance.x;
        result_msg.r_elbow_match_quality.y = pose_error.r_elbow.getY() / variance.y;
        result_msg.r_elbow_match_quality.z = pose_error.r_elbow.getZ() / variance.z;

        // complete action
        _EvaluatePoseActionServerPolicy::completeAction( result_msg );
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( evaluatePoseActionExecuteCB, _EvaluatePoseAction )
    {
        if( !_EvaluatePoseActionServerPolicy::waitOnAction( 2 ) ) return;
    }
};

#endif // ARMPOSERECOGNIZER_ARMPOSERECOGNIZERNODE_H_
