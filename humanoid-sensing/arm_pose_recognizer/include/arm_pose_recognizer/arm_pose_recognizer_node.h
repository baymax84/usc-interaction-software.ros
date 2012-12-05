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

// policies
#include <quickdev/action_server_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

// objects
#include <arm_pose_recognizer/pose.h>
#include <arm_pose_recognizer/meta_joint.h>

// utils
#include <quickdev/geometry_message_conversions.h>

// actions
#include <arm_pose_recognizer/EvaluatePoseAction.h>

typedef arm_pose_recognizer::EvaluatePoseAction _EvaluatePoseAction;
typedef quickdev::ActionServerPolicy<_EvaluatePoseAction> _EvaluatePoseActionServerPolicy;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

typedef MetaJoint _MetaJoint;
typedef std::map<std::string, _MetaJoint> _UnitHumanoid;

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

    // climb our unit-humanoid "tree" from the start to the end (or topmost parent) joint, calculating the cumulative transform as we go
    static btTransform lookupUnitHumanoidTransform( _UnitHumanoid const & unit_humanoid, std::string const & start, std::string const & end = "" )
    {
        // start with a zero transform
        btTransform cumulative_tf( btQuaternion( 0, 0, 0, 1 ) );

        // initialize our search with the starting joint
        auto unit_humanoid_it = unit_humanoid.find( start );

        // while the joint we're looking for exists
        while( unit_humanoid_it != unit_humanoid.cend() )
        {
            // get a ref to the meta-joint
            auto const & meta_joint = unit_humanoid_it->second;

            // transform the cumulative tf by this meta-joint's inverse tf (since we're climbing up the tree, but the tfs are defined in the
            // opposite order
            cumulative_tf *= meta_joint.relative_transform_.inverse();

            // if we've reached the given end joint, we're done traversing the tree
            if( meta_joint.parent_name_ == end ) break;

            // otherwise, grab the parent meta-joint
            unit_humanoid_it = unit_humanoid.find( meta_joint.parent_name_ );
        }

        return cumulative_tf;
    }

    // This optional function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate).
    // Most updateable policies should have their update( ... ) functions called within this context.
    //
    QUICKDEV_SPIN_ONCE()
    {
        if( !_EvaluatePoseActionServerPolicy::active() ) return;

        // look up tf frames

        auto const & goal = _EvaluatePoseActionServerPolicy::getGoal();

        auto const & meta_joints = goal.meta_joints;
        auto const & desired_joint_names = goal.desired_joint_names;
        auto const & observed_joint_names = goal.observed_joint_names;
        auto const & variance = goal.variance;

        if( desired_joint_names.size() != observed_joint_names.size() )
        {
            PRINT_ERROR( "The number of desired meta-joints must be the same as the number of observed meta-joints; aborting action." );
            return abortAction();
        }

        auto desired_joint_name_it = desired_joint_names.cbegin();
        auto observed_joint_name_it = observed_joint_names.cbegin();

        // build meta-joints by calculating rotation component of transform from start to end frame
        // our unit-humanoid is just a map of these joints indexed by joint name
        _UnitHumanoid unit_humanoid;

        for( auto meta_joint_it = meta_joints.cbegin(); meta_joint_it != meta_joints.cend(); ++meta_joint_it )
        {
            auto const & meta_joint_msg = *meta_joint_it;
            auto const chain_tf = _TfTranceiverPolicy::lookupTransform( meta_joint_msg.start_frame_name, meta_joint_msg.end_frame_name );
            unit_humanoid[meta_joint_msg.name] = _MetaJoint( chain_tf.getRotation(), meta_joint_msg.name, meta_joint_msg.parent_name );
        }

        _EvaluatePoseActionServerPolicy::_ResultMsg result_msg;

        // using our unit-humanoid and our lookupUnitHumanoidTransform function, calculate the distances between all the given meta-joints
        btVector3 const variance_vec = unit::implicit_convert( variance );
        for( ; desired_joint_name_it != desired_joint_names.cend(); ++desired_joint_name_it, ++observed_joint_name_it )
        {
            btTransform desired_tf = lookupUnitHumanoidTransform( unit_humanoid, *desired_joint_name_it );
            btTransform observed_tf = lookupUnitHumanoidTransform( unit_humanoid, *observed_joint_name_it );

            result_msg.match_qualities.push_back( unit::implicit_convert( ( observed_tf.getOrigin() - desired_tf.getOrigin() ) / variance_vec ) );
        }

        // complete action
        _EvaluatePoseActionServerPolicy::completeAction( result_msg );
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( evaluatePoseActionExecuteCB, _EvaluatePoseAction )
    {
        if( !_EvaluatePoseActionServerPolicy::waitOnAction( 2 ) ) return;
    }
};

#endif // ARMPOSERECOGNIZER_ARMPOSERECOGNIZERNODE_H_
