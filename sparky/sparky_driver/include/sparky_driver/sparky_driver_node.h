/***************************************************************************
 *  include/sparky_driver/sparky_driver_node.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski (ekaszubski@gmail.com)
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
 *  * Neither the name of sparky nor the names of its
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

#ifndef SPARKYDRIVER_SPARKYDRIVERNODE_H_
#define SPARKYDRIVER_SPARKYDRIVERNODE_H_

#include <quickdev/node.h>

// objects
//#include <sparky/sparky_controller.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

// msgs
#include <sensor_msgs/JointState.h>

// Declare a node called SparkyDriverNode.
// A quickdev::RunablePolicy is automatically prepended to the list of policies our node will use.
// To use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( SparkyDriver, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( SparkyDriver )

typedef sensor_msgs::JointState _JointStateMsg;
typedef robot_state_publisher::RobotStatePublisher _RobotStatePublisher;

//using humanoid::_Humanoid;
//using humanoid::_HumanoidStateArrayMsg;
//using humanoid::_HumanoidStateMsg;
//using humanoid::_HumanoidJointMsg;
//using humanoid::_JointName;

// Declare a class called SparkyDriverNode
//
QUICKDEV_DECLARE_NODE_CLASS( SparkyDriver )
{
protected:
    ros::MultiSubscriber<> multi_sub_;
    KDL::Tree sparky_model_;
    boost::shared_ptr<_RobotStatePublisher> sparky_state_pub_ptr_;
    std::map<std::string, double> joint_states_map_;
    bool simulate_;

//    sparky::SparkyController sparky_controller_;

    // Variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( SparkyDriver ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( SparkyDriver )
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
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "joint_angles", &SparkyDriverNode::jointAnglesCB, this );
        multi_sub_.addSubscriber( nh_rel, "servo_angles", &SparkyDriverNode::servoAnglesCB, this );

        simulate_ = quickdev::ParamReader::readParam<bool>( nh_rel, "simulate", false );

        // -------------------------------------------------------------------------------------------------------------------------------------
        // load URDF and set up robot state publisher
        auto const urdf_filename = quickdev::ParamReader::readParam<std::string>( nh_rel, "urdf_filename", "" );

        if( !kdl_parser::treeFromFile( urdf_filename, sparky_model_ ) )
        {
            ROS_ERROR( "Failed to construct kdl tree" );
            return QUICKDEV_GET_RUNABLE_POLICY()::interrupt();
        }

        auto const & segments_map = sparky_model_.getSegments();
        ROS_INFO( "Initializing segments..." );
        for( auto segment_it = segments_map.cbegin(); segment_it != segments_map.cend(); ++segment_it )
        {
            auto const & tree_elem = segment_it->second;
            auto const & parent_segment_it = tree_elem.parent;
//            if( parent_segment_it != segments_map.cend() )
//            {
//                ROS_INFO( "Segment: [ %s ] <- [ %s ]", parent_segment_it->first.c_str(), segment_it->first.c_str() );
//            }
//            else ROS_INFO( "Segment: [ %s ]", segment_it->first.c_str() );

            ROS_INFO( "Joint: [ %s ]", tree_elem.segment.getJoint().getName().c_str() );
            for( size_t i = 0; i < tree_elem.children.size(); ++i )
            {
                auto const & child_segment_it = tree_elem.children[i];
                if( child_segment_it == segments_map.cend() ) continue;
                auto const & child_tree_elem = child_segment_it->second;
                auto const & child_parent_segment_it = child_tree_elem.parent;
                ROS_INFO( "Child segment: [ %s ] <- [ %s ]", child_parent_segment_it->first.c_str(), child_segment_it->first.c_str() );
                ROS_INFO( "Child joint: [ %s ]", child_tree_elem.segment.getJoint().getName().c_str() );
            }
            joint_states_map_[tree_elem.segment.getJoint().getName()] = 0;
        }

        sparky_state_pub_ptr_ = boost::make_shared<_RobotStatePublisher>( sparky_model_ );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( jointAnglesCB, _JointStateMsg )
    {
        auto const & names = msg->name;
        auto const & positions = msg->position;

        auto name_it = names.cbegin();
        auto position_it = positions.cbegin();

        for( ; name_it != names.cend() && position_it != positions.cend(); ++name_it, ++position_it )
        {
            joint_states_map_[*name_it] = *position_it;
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( servoAnglesCB, _JointStateMsg )
    {
        //
    }

    // This optional function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate).
    // Most updateable policies should have their update( ... ) functions called within this context.
    //
    QUICKDEV_SPIN_ONCE()
    {
        ros::Time const now = ros::Time::now();
//        for( auto joint_state_it = joint_states_map_.cbegin(); joint_state_it != joint_states_map_.cend(); ++joint_state_it )
//        {
//            ROS_INFO( "[ %s ] : [ %f ]", joint_state_it->first.c_str(), joint_state_it->second );
//        }
        sparky_state_pub_ptr_->publishTransforms( joint_states_map_, now );
        sparky_state_pub_ptr_->publishFixedTransforms();
    }
};

#endif // SPARKYDRIVER_SPARKYDRIVERNODE_H_
