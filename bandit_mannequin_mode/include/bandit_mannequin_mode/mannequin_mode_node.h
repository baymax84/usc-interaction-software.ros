/***************************************************************************
 *  include/bandit_mannequin_mode/mannequin_mode_node.h
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

#ifndef BANDITMANNEQUINMODE_MANNEQUINMODENODE_H_
#define BANDITMANNEQUINMODE_MANNEQUINMODENODE_H_

#include <quickdev/node.h>

#include <bandit/joint_name.h>

// objects
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

// msgs
#include <sensor_msgs/JointState.h>

typedef sensor_msgs::JointState _JointStateMsg;

QUICKDEV_DECLARE_NODE( MannequinMode )

QUICKDEV_DECLARE_NODE_CLASS( MannequinMode )
{
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    size_t const left_wrist_hand_joint_id_;
    size_t const right_torso_shoulder_mounting_joint_id_;
    size_t const right_wrist_hand_joint_id_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( MannequinMode ),
        left_wrist_hand_joint_id_( bandit::JointName( "left_wrist_hand_joint" ).id_ ),
        right_torso_shoulder_mounting_joint_id_( bandit::JointName( "right_torso_shoulder_mounting_joint" ).id_ ),
        right_wrist_hand_joint_id_( bandit::JointName( "right_wrist_hand_joint" ).id_ )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_JointStateMsg>( nh_rel, { "joint_state_command" } );
        multi_sub_.addSubscriber( nh_rel, "joint_states", &MannequinModeNode::jointStatesCB, this );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
/*
        _JointStateMsg joint_states;
        joint_states.name.push_back( "head_pan_joint" );
        joint_states.position.push_back( M_PI_2 * sin( ( ros::Time::now() - start_time_ ).toSec() ) );
        multi_pub_.publish( "joint_state_command", joint_states );
*/
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( jointStatesCB, _JointStateMsg )
    {
        _JointStateMsg joint_state_msg;
        joint_state_msg.header = msg->header;
        joint_state_msg.position.resize( 12 );
        joint_state_msg.name.resize( 12 );

        auto position_output_iterator = joint_state_msg.position.begin();
        auto name_output_iterator = joint_state_msg.name.begin();

        position_output_iterator = std::copy( msg->position.begin()                                          , msg->position.begin() + left_wrist_hand_joint_id_ , position_output_iterator );
        position_output_iterator = std::copy( msg->position.begin() + right_torso_shoulder_mounting_joint_id_, msg->position.begin() + right_wrist_hand_joint_id_, position_output_iterator );

        name_output_iterator = std::copy( msg->name.begin()                                          , msg->name.begin() + left_wrist_hand_joint_id_ , name_output_iterator );
        name_output_iterator = std::copy( msg->name.begin() + right_torso_shoulder_mounting_joint_id_, msg->name.begin() + right_wrist_hand_joint_id_, name_output_iterator );

        multi_pub_.publish( "joint_state_command", quickdev::make_const_shared( joint_state_msg ) );
    }
};

#endif // BANDITMANNEQUINMODE_MANNEQUINMODENODE_H_
