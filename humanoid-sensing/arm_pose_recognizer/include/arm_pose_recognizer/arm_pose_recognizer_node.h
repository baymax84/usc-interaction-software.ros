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

typedef humanoid_sensing_msgs::MetaJoint _MetaJointMsg;

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
        PRINT_INFO( "Looking up unit-humanoid transform [ %s ] -> [ %s ]", start.c_str(), end.c_str() );

        // start with a zero transform
        btTransform cumulative_translation_tf( btQuaternion( 0, 0, 0, 1 ) );

        // initialize our search with the starting joint
        auto unit_humanoid_it = unit_humanoid.find( start );

        // while the joint we're looking for exists
        while( unit_humanoid_it != unit_humanoid.cend() )
        {
            // get a ref to the meta-joint
            auto const & meta_joint = unit_humanoid_it->second;

            // split up the current tf (which will necessarily be normalized) into its translation and rotation components
//            tf::Transform const & current_tf = meta_joint.internal_transform_;
//            tf::Transform const current_rotation_tf = btTransform( current_tf.getRotation() );
//            tf::Transform const current_translation_tf = btTransform( btQuaternion( 0, 0, 0, 1 ), current_tf.getTranslation() );

            // transform the current joint's translation into the parent frame's coordinate frame using the inverse of the cumulative rotation transform
            // additionally, since the
//            cumulative_translation_tf = current_translation_tf * ( current_rotation_tf * cumulative_translation_tf );

            // if we've reached the given end joint, we're done traversing the tree
            if( meta_joint.parent_name_ == end )
            {
                PRINT_INFO( "Reached end of chain" );
                break;
            }

            auto const & cumulative_translation = cumulative_translation_tf.getOrigin();
            auto const & current_translation = meta_joint.transform_to_parent_.getOrigin();
            PRINT_INFO( "[ %s ] -> [ %s ] ( %f, %f, %f ) * ( %f, %f, %f )", meta_joint.name_.c_str(), meta_joint.parent_name_.c_str(), cumulative_translation.x(), cumulative_translation.y(), cumulative_translation.z(), current_translation.x(), current_translation.y(), current_translation.z() );

            // otherwise, update the cumulative translation tf
            cumulative_translation_tf *= meta_joint.transform_to_parent_;
            // and grab the parent meta-joint
            unit_humanoid_it = unit_humanoid.find( meta_joint.parent_name_ );
        }

        auto const & cumulative_translation = cumulative_translation_tf.getOrigin();
        PRINT_INFO( "Cumulative translation: ( %f, %f, %f )", cumulative_translation.x(), cumulative_translation.y(), cumulative_translation.z() );

        return cumulative_translation_tf;
    }

    static _MetaJointMsg const & findTopmostParentJoint( std::string const & joint, std::map<std::string, _MetaJointMsg> const & meta_joint_map )
    {
        PRINT_INFO( "Looking for topmost parent joint of [ %s ]", joint.c_str() );
        auto current_meta_joint_it = meta_joint_map.find( joint );
        while( true )
        {
            auto const & meta_joint = current_meta_joint_it->second;
            auto new_meta_joint_it = meta_joint_map.find( meta_joint.parent_name );

            if( new_meta_joint_it == meta_joint_map.cend() )
            {
                break;
            }

            current_meta_joint_it = new_meta_joint_it;
        }

        auto const & meta_joint = current_meta_joint_it->second;

        PRINT_INFO( "Found [ %s ]", meta_joint.name.c_str() );

        return meta_joint;
    }


    // This optional function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate).
    // Most updateable policies should have their update( ... ) functions called within this context.
    //
    QUICKDEV_SPIN_ONCE()
    {
        if( !_EvaluatePoseActionServerPolicy::active() ) return;

        // look up tf frames

        auto const & goal = _EvaluatePoseActionServerPolicy::getGoal();

        auto const & desired_meta_joints = goal.desired_meta_joints;
        auto const & observed_meta_joints = goal.observed_meta_joints;
        auto const & desired_joint_names = goal.desired_joint_names;
        auto const & observed_joint_names = goal.observed_joint_names;
        auto const & variance = goal.variance;
        auto const & root_desired_frame = goal.root_desired_frame;
        auto const & root_observed_frame = goal.root_observed_frame;
        //auto const root_desired_frame = root_desired_joint.data;
        //auto const root_observed_frame = root_observed_joint.data;

        if( desired_joint_names.size() != observed_joint_names.size() )
        {
            PRINT_ERROR( "The number of desired meta-joints must be the same as the number of observed meta-joints; aborting action." );
            return abortAction();
        }

        // build meta-joints by calculating rotation component of transform from start to end frame
        // our unit-humanoid is just a map of these joints indexed by joint name
        // note: this data structure holds the unit-vectors for *both* the observed and desired humanoid
        _UnitHumanoid unit_humanoid;
        std::map<std::string, _MetaJointMsg> meta_joint_map;

        std::map<std::string, _MetaJointMsg> desired_meta_joint_map;
        std::map<std::string, _MetaJointMsg> observed_meta_joint_map;

        // build meta-joint map
        PRINT_INFO( "Building meta-joint map" );
        for( auto meta_joint_it = desired_meta_joints.cbegin(); meta_joint_it != desired_meta_joints.cend(); ++meta_joint_it )
        {
            auto const & meta_joint_msg = *meta_joint_it;
            PRINT_INFO( "Saving meta-joint with name [ %s ]", meta_joint_msg.name.c_str() );
            meta_joint_map[meta_joint_msg.name] = meta_joint_msg;
            desired_meta_joint_map[meta_joint_msg.name] = meta_joint_msg;
        }
        for( auto meta_joint_it = observed_meta_joints.cbegin(); meta_joint_it != observed_meta_joints.cend(); ++meta_joint_it )
        {
            auto const & meta_joint_msg = *meta_joint_it;
            PRINT_INFO( "Saving meta-joint with name [ %s ]", meta_joint_msg.name.c_str() );
            meta_joint_map[meta_joint_msg.name] = meta_joint_msg;
            observed_meta_joint_map[meta_joint_msg.name] = meta_joint_msg;
        }

        PRINT_INFO( "Root meta-joints; desired: [ %s ] observed: [ %s ]", root_desired_frame.c_str(), root_observed_frame.c_str() );

        // map from joint name to topmost-parent meta-joint
        std::map<std::string, _MetaJointMsg> root_meta_joint_map;
        for( auto meta_joint_it = meta_joint_map.cbegin(); meta_joint_it != meta_joint_map.cend(); ++meta_joint_it )
        {
            auto const & meta_joint_msg = meta_joint_it->second;
            root_meta_joint_map[meta_joint_msg.name] = findTopmostParentJoint( meta_joint_msg.name, meta_joint_map );
        }

        // a map of transforms from the root meta joint's end frame to the frame stored in the key
        std::map<std::string, btTransform> transforms_map;
        transforms_map[root_desired_frame] = btTransform( btQuaternion( 0, 0, 0, 1 ) );
        transforms_map[root_observed_frame] = btTransform( btQuaternion( 0, 0, 0, 1 ) );

        // look up all the transforms we'll need, relative to topmost parent; we may not actually use all of these depending on the meta-joints
        PRINT_INFO( "Looking up transforms" );
        for( auto meta_joint_it = meta_joint_map.cbegin(); meta_joint_it != meta_joint_map.cend(); ++meta_joint_it )
        {
            auto const & meta_joint_msg = meta_joint_it->second;
            // we want a transform without any rotation here

            auto const & joint_start_to_frame = meta_joint_msg.start_frame_name;
            auto const & joint_end_to_frame = meta_joint_msg.end_frame_name;

            // if a root desired/observed joint name is not defined, use the calculated root from our root_meta_joint map
            std::string joint_from_frame;

            bool const is_desired_joint = desired_meta_joint_map.count( meta_joint_msg.name ) > 0;
            bool const is_observed_joint = observed_meta_joint_map.count( meta_joint_msg.name ) > 0;

            if( ( is_desired_joint && root_desired_frame.empty() ) || ( is_observed_joint && root_observed_frame.empty() ) )
            {
                joint_from_frame = root_meta_joint_map.find( meta_joint_msg.name )->second.end_frame_name;
            }
            // otherwise, use the given frame as the transform source
            else
            {
                if( is_desired_joint && !is_observed_joint ) joint_from_frame = root_desired_frame;
                if( !is_desired_joint && is_observed_joint ) joint_from_frame = root_observed_frame;
            }

            PRINT_INFO( "Saving transform [ %s ] -> [ %s ]", joint_from_frame.c_str(), joint_start_to_frame.c_str() );
            PRINT_INFO( "Saving transform [ %s ] -> [ %s ]", joint_from_frame.c_str(), joint_end_to_frame.c_str() );
            transforms_map[joint_start_to_frame] = btTransform( btQuaternion( 0, 0, 0, 1 ), _TfTranceiverPolicy::lookupTransform( joint_from_frame, joint_start_to_frame ).getOrigin() );
            transforms_map[joint_end_to_frame] = btTransform( btQuaternion( 0, 0, 0, 1 ), _TfTranceiverPolicy::lookupTransform( joint_from_frame, joint_end_to_frame ).getOrigin() );
        }

        // pull transforms and build our unit-humanoids
        PRINT_INFO( "Building unit humanoids" );
        for( auto meta_joint_it = meta_joint_map.cbegin(); meta_joint_it != meta_joint_map.cend(); ++meta_joint_it )
        {
            auto const & meta_joint_msg = meta_joint_it->second;

            // the iterator for the parent meta-joint
            auto const parent_meta_joint_it = meta_joint_map.find( meta_joint_msg.parent_name );

            auto parent_tf = btTransform( btQuaternion( 0, 0, 0, 1 ) );

            // does the parent exist?
            if( parent_meta_joint_it != meta_joint_map.cend() )
            {
                auto const & parent_meta_joint_msg = parent_meta_joint_it->second;
                // the "parent transform" is the transform from the child's start frame to the parent's end frame (ie it points back up to the parent,
                // whereas the "internal transform" points in the opposite direction)
                parent_tf = transforms_map[parent_meta_joint_msg.end_frame_name].inverse() * transforms_map[meta_joint_msg.start_frame_name];
//                parent_tf = _TfTranceiverPolicy::lookupTransform( meta_joint_msg.start_frame_name, parent_meta_joint_msg.end_frame_name );
            }

            PRINT_INFO( "Adding entry to unit-humanoid: [ %s ] -> [ %s ]", meta_joint_msg.name.c_str(), meta_joint_msg.parent_name.c_str() );
            unit_humanoid[meta_joint_msg.name] = _MetaJoint( parent_tf, meta_joint_msg.name, meta_joint_msg.parent_name );
        }

        _EvaluatePoseActionServerPolicy::_ResultMsg result_msg;

        // using our unit-humanoid and our lookupUnitHumanoidTransform function, calculate the distances between all the given meta-joints
        btVector3 const variance_vec = unit::implicit_convert( variance );
        auto desired_joint_name_it = desired_joint_names.cbegin();
        auto observed_joint_name_it = observed_joint_names.cbegin();
        for( ; desired_joint_name_it != desired_joint_names.cend(); ++desired_joint_name_it, ++observed_joint_name_it )
        {
            auto const & desired_joint_name = *desired_joint_name_it;
            auto const & observed_joint_name = *observed_joint_name_it;
            btTransform desired_tf = lookupUnitHumanoidTransform( unit_humanoid, desired_joint_name );
            btTransform observed_tf = lookupUnitHumanoidTransform( unit_humanoid, observed_joint_name );

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
