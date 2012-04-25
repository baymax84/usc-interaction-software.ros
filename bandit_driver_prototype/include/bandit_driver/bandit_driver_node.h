/***************************************************************************
 *  include/bandit_driver/bandit_driver_node.h
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

#ifndef BANDITDRIVERPROTOTYPE_BANDITDRIVER_BANDITDRIVERNODE_H_
#define BANDITDRIVERPROTOTYPE_BANDITDRIVER_BANDITDRIVERNODE_H_

#include <quickdev/node.h>

// objects
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <bandit/bandit.h>
//#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.h>


// messages
#include <bandit_msgs/JointArray.h>
#include <sensor_msgs/JointState.h>

// others
#include <quickdev/geometry_message_conversions.h>
#include <kdl_parser/kdl_parser.hpp>

typedef bandit_msgs::JointArray _BanditJointArrayMsg;
typedef sensor_msgs::JointState _JointStateMsg;

// Declare a node called BanditDriverNode.
// A quickdev::RunablePolicy is automatically prepended to the list of policies our node will use.
// To use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( BanditDriver, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( BanditDriver )

// Declare a class called BanditDriverNode
//
QUICKDEV_DECLARE_NODE_CLASS( BanditDriver )
{
    typedef robot_state_publisher::RobotStatePublisher _RobotStatePublisher;

    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    bandit::Bandit bandit_driver_;
    KDL::Tree bandit_model_;
    boost::shared_ptr<_RobotStatePublisher> bandit_state_pub_ptr_;

    // Variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( BanditDriver ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( BanditDriver )
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

        auto const urdf_filename = ros::ParamReader<std::string, 1>::readParam( nh_rel, "urdf_filename", "" );

//        bandit_model_.initFile( urdf_filename );

        if( !kdl_parser::treeFromFile( urdf_filename, bandit_model_ ) )
        {
            ROS_ERROR( "Failed to construct kdl tree" );
            QUICKDEV_GET_RUNABLE_POLICY()::interrupt();
        }

        bandit_state_pub_ptr_ = quickdev::make_shared( new _RobotStatePublisher( bandit_model_ ) );

        auto const port = ros::ParamReader<std::string, 1>::readParam( nh_rel, "port", "/dev/ttyUSB0" );

        bandit_driver_.openPort( port.c_str() );
        bandit_driver_.registerStateCB( quickdev::auto_bind( &BanditDriverNode::banditStateCB, this ) );

        multi_pub_.addPublishers<_JointStateMsg>( nh_rel, { "joint_state" } );

        multi_sub_.addSubscriber( nh_rel, "joint_command", &BanditDriverNode::jointCommandCB, this );

        initPolicies<quickdev::policy::ALL>();
    }

    // This opitonal function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate).
    // Most updateable policies should have their update( ... ) functions called within this context.
    //
    QUICKDEV_SPIN_ONCE()
    {
        //
    }

    void banditStateCB()
    {
        // let's assume we're not going to change the number of joints or their name mappings after initialization
        static const size_t num_joints = bandit_driver_.getNumJoints();
        // lookup the eyebrow joint to prevent doing this for every joint every function call
        static const size_t eyebrows_joint_index = bandit_driver_.getJointIndexByROSName( "eyebrows_joint" );
        _JointStateMsg joint_state_msg;

        joint_state_msg.name.reserve( num_joints );
        joint_state_msg.position.reserve( num_joints );

        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.header.frame_id = "bandit_torso_link";

        std::map<std::string, double> joints_map;

        for ( size_t i = 0; i < num_joints; ++i )
        {
            auto const joint_name = bandit_driver_.getJointRosName( i );
            auto const joint_pos = bandit_driver_.getJointPos( i );
            // the eyebrows are really two joints as far as robot_state_publisher is concerned
            if ( i == eyebrows_joint_index )
            {
                joint_state_msg.name.push_back( std::string( "bandit_head_left_brow_joint" ) );
                joint_state_msg.position.push_back( -3 * joint_pos );
                joint_state_msg.name.push_back( std::string( "bandit_head_right_brow_joint" ) );
                joint_state_msg.position.push_back( -3 * joint_pos );
            }
            else
            {
                joint_state_msg.name.push_back( joint_name );
                joint_state_msg.position.push_back( joint_pos );
            }

            joints_map[joint_name] = joint_pos;
        }

        bandit_state_pub_ptr_->publishTransforms( joints_map, ros::Time::now() );

        multi_pub_.publish( "joint_state", quickdev::make_const_shared( joint_state_msg ) );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( jointCommandCB, _BanditJointArrayMsg )
    {
        auto const & joints = msg->joints;

        for( auto joints_it = joints.cbegin(); joints_it != joints.cend(); ++joints_it )
        {
            auto const & joint_id = joints_it->id;
            auto const & joint_angle = joints_it->angle;

            bandit_driver_.setJointPos( joint_id, joint_angle );
        }

        bandit_driver_.sendAllJointPos();
    }
};

#endif // BANDITDRIVERPROTOTYPE_BANDITDRIVER_BANDITDRIVERNODE_H_
