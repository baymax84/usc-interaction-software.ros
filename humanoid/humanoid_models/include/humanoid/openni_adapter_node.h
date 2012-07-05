/***************************************************************************
 *  include/humanoid/openni_adapter_node.h
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

#ifndef HUMANOIDMODELS_HUMANOID_OPENNIADAPTERNODE_H_
#define HUMANOIDMODELS_HUMANOID_OPENNIADAPTERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/tf_tranceiver_policy.h>
#include <quickdev/timed_policy.h>

// objects
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

// utils
#include <quickdev/threading.h>
#include <quickdev/geometry_message_conversions.h>
#include <quickdev/math.h>
#include <humanoid/humanoid_features.h>

// msgs
#include <openni_multitracker/UserStateArray.h>

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef quickdev::TimedPolicy<> _UserStatesCBTimer;
QUICKDEV_DECLARE_NODE( OpenNIAdapter, _TfTranceiverPolicy, _UserStatesCBTimer )

using humanoid::_HumanoidStateArrayMsg;
using humanoid::_HumanoidStateMsg;
using humanoid::_HumanoidJointMsg;
using humanoid::_JointName;
typedef std::map<_JointName, btQuaternion> _JointNormMap;
typedef std::map<_JointName, std::vector<double> > _JointErrorMap;

static _JointErrorMap generateJointErrorMap()
{
    _JointErrorMap joint_error_map;

    // joint_error_map["joint"]            = { r, p, y };
    joint_error_map["head"]             = { 0, 0, 0 };
    joint_error_map["neck"]             = { 0, 0, 0 };
    joint_error_map["torso"]            = { 0, 0, 0 };
    joint_error_map["waist"]            = { 0, 0, 0 };
    joint_error_map["right_collar"]     = { 0, 0, 0 };
    joint_error_map["right_shoulder"]   = { 0, 0, 0 };
    joint_error_map["right_elbow"]      = { 0, 0, 0 };
    joint_error_map["right_wrist"]      = { 0, 0, 0 };
    joint_error_map["right_hand"]       = { 0, 0, 0 };
    joint_error_map["right_finger_tip"] = { 0, 0, 0 };
    joint_error_map["left_collar"]      = { 0, 0, 0 };
    joint_error_map["left_shoulder"]    = { 0, 0, 0 };
    joint_error_map["left_elbow"]       = { 0, 0, 0 };
    joint_error_map["left_wrist"]       = { 0, 0, 0 };
    joint_error_map["left_hand"]        = { 0, 0, 0 };
    joint_error_map["left_finger_tip"]  = { 0, 0, 0 };
    joint_error_map["right_hip"]        = { 0, 0, 0 };
    joint_error_map["right_knee"]       = { 0, 0, 0 };
    joint_error_map["right_ankle"]      = { 0, 0, 0 };
    joint_error_map["right_foot"]       = { 0, 0, 0 };
    joint_error_map["left_hip"]         = { 0, 0, 0 };
    joint_error_map["left_knee"]        = { 0, 0, 0 };
    joint_error_map["left_ankle"]       = { 0, 0, 0 };
    joint_error_map["left_foot"]        = { 0, 0, 0 };

    return joint_error_map;
}

static _JointNormMap generateJointNormMap()
{
    _JointNormMap joint_norm_map;

    joint_norm_map["head"]             = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["neck"]             = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["torso"]            = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["waist"]            = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_collar"]     = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_shoulder"]   = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( -180 ) ) );
    joint_norm_map["right_elbow"]      = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 0  ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_wrist"]      = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_hand"]       = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_finger_tip"] = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_collar"]      = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_shoulder"]    = btQuaternion( Radian( Degree( 90  ) ), Radian( Degree( 90 ) ), Radian( Degree( -180 ) ) );
    joint_norm_map["left_elbow"]       = btQuaternion( Radian( Degree( 90  ) ), Radian( Degree( 0  ) ), Radian( Degree( -180 ) ) );
    joint_norm_map["left_wrist"]       = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_hand"]        = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_finger_tip"]  = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_hip"]        = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["right_knee"]       = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["right_ankle"]      = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["right_foot"]       = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_hip"]         = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["left_knee"]        = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["left_ankle"]       = btQuaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["left_foot"]        = btQuaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );

    return joint_norm_map;
}

static auto getJointNormMap() -> decltype( generateJointNormMap() ) const &
{
    static auto const & joint_norm_map = generateJointNormMap();

    return joint_norm_map;
}

static auto getJointErrorMap() -> decltype( generateJointErrorMap() ) const &
{
    static auto const & joint_error_map = generateJointErrorMap();

    return joint_error_map;
}

//! Reads data from an openni_multitracker/openni_multitracker node and converts these data into Humanoid messages
QUICKDEV_DECLARE_NODE_CLASS( OpenNIAdapter )
{
private:
    typedef openni_multitracker::UserStateArray _UserStateArrayMsg;

    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    quickdev::MessageCache<_UserStateArrayMsg> user_states_cache_;

    std::string world_frame_;
    ros::Duration kinect_timeout_duration_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( OpenNIAdapter ),
        world_frame_( "/world" ),
        kinect_timeout_duration_( 1.0 )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
        multi_pub_.addPublishers<_HumanoidStateArrayMsg>( nh_rel, { "humanoid_states" } );
        multi_sub_.addSubscriber( nh_rel, "user_states", &OpenNIAdapterNode::userStatesCB, this );
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const now = ros::Time::now();
        if( now - _UserStatesCBTimer::last() > kinect_timeout_duration_ ) return;

        QUICKDEV_LOCK_CACHE_AND_GET( user_states_cache_, user_states_msg );
        if( !user_states_msg ) return;

        _HumanoidStateArrayMsg state_array_msg;

        state_array_msg.header.stamp = now;

        auto const & joint_norm_map = getJointNormMap();
        auto const & joint_error_map = getJointErrorMap();
        auto const & joint_dependency_map = humanoid::getJointDependencyMap();

        for( auto user_state = user_states_msg->user_states.cbegin(); user_state != user_states_msg->user_states.cend(); ++user_state )
        {
            if( !user_state->is_tracked ) continue;

            _HumanoidStateMsg state_msg;
            state_msg.header.stamp = now;

            state_msg.name = user_state->name;

            std::map<std::string, _HumanoidJointMsg> joint_msg_map;

            for( auto joint_name_it = humanoid::JOINT_NAMES_.begin(); joint_name_it != humanoid::JOINT_NAMES_.end(); ++joint_name_it )
            {
                auto const & joint_name = *joint_name_it;
                auto const parent_joint_name = joint_dependency_map.find( joint_name )->second;

                auto const joint_frame_name( "/" + user_state->name + "/" + joint_name );
                //auto const parent_joint_frame_name( "/" + user_state->name + "/" + parent_joint_name );

                if( transformExists( world_frame_, joint_frame_name ) )
                {
                    auto const world_transform( lookupTransform( world_frame_, joint_frame_name ) );
                    //auto const parent_transform( lookupTransform( parent_joint_frame_name, joint_frame_name ) );

                    auto const norm_rotation = joint_norm_map.find( joint_name )->second;
                    btTransform const normalized_transform( world_transform.getRotation() * norm_rotation, world_transform.getOrigin() );

                    _HumanoidJointMsg joint_msg;

                    joint_msg.name = joint_name;
                    joint_msg.parent_name = parent_joint_name;

                    auto const normalized_position = normalized_transform.getOrigin();
                    auto const normalized_rotation = normalized_transform.getRotation();

                    joint_msg.pose.pose.position = unit::make_unit( normalized_position );
                    joint_msg.pose.pose.orientation = unit::make_unit( normalized_rotation );

                    // joint_msg.pose.covariance[row * width + col]
                    double const position_sigma = pow( normalized_position.length() - 0.039, 2 ) * 0.0055;
                    joint_msg.pose.covariance[0 * 6 + 0] = position_sigma;
                    joint_msg.pose.covariance[1 * 6 + 1] = position_sigma;
                    joint_msg.pose.covariance[2 * 6 + 2] = position_sigma;

                    auto joint_error_model_it = joint_error_map.find( joint_name );
                    if( joint_error_model_it != joint_error_map.end() )
                    {
                        auto const & joint_error_model = joint_error_model_it->second;
                        joint_msg.pose.covariance[3 * 6 + 3] = joint_error_model[3];
                        joint_msg.pose.covariance[4 * 6 + 4] = joint_error_model[4];
                        joint_msg.pose.covariance[5 * 6 + 5] = joint_error_model[5];
                    }

                    joint_msg_map[joint_name] = joint_msg;
                    publishTransform( normalized_transform, world_frame_, joint_frame_name + "_norm" );
                }
            }

            // head given waist
            {
                auto parent_joint_msg_it = joint_msg_map.find( "waist" );
                auto joint_msg_it = joint_msg_map.find( "head" );

                // if the parent joint doesn't exist, then we can't do anything here
                if( parent_joint_msg_it != joint_msg_map.end() && joint_msg_it != joint_msg_map.end() )
                {
                    auto & parent_joint_msg = parent_joint_msg_it->second;
                    auto & joint_msg = joint_msg_it->second;

                    joint_msg.pose.covariance[5 * 6 + 5] = quickdev::gaussian_product_variance( parent_joint_msg.pose.covariance[5 * 6 + 5], joint_msg.pose.covariance[5 * 6 + 5] );
                }
            }

            // eyes given head
            {
                auto parent_joint_msg_it = joint_msg_map.find( "head" );

                // if the parent joint doesn't exist, then we can't do anything here
                if( parent_joint_msg_it != joint_msg_map.end() )
                {
                    auto & parent_joint_msg = parent_joint_msg_it->second;

                    _HumanoidJointMsg joint_msg;

                    joint_msg.name = "eyes";
                    joint_msg.parent_name = "head";

                    // copy parent pose
                    joint_msg.pose = parent_joint_msg.pose;

                    joint_msg.pose.pose.position.x += 0; // some constant(?) offset
                    joint_msg.pose.pose.position.y += 0; // some constant(?) offset
                    joint_msg.pose.pose.position.z += 0; // some constant(?) offset

                    joint_msg.pose.covariance[5 * 6 + 5] = quickdev::gaussian_product_variance( parent_joint_msg.pose.covariance[5 * 6 + 5], joint_msg.pose.covariance[5 * 6 + 5] );
                }
            }

            for( auto joint_msg_it = joint_msg_map.cbegin(); joint_msg_it != joint_msg_map.cend(); ++joint_msg_it )
            {
                state_msg.joints.push_back( joint_msg_it->second );
            }

            state_array_msg.states.push_back( state_msg );
        }

        multi_pub_.publish( "humanoid_states", quickdev::make_const_shared( state_array_msg ) );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( userStatesCB, _UserStateArrayMsg )
    {
        _UserStatesCBTimer::update();

        QUICKDEV_TRY_UPDATE_CACHE( user_states_cache_, msg );
        QUICKDEV_TRY_LOCK_OR_WARN( user_states_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );
    }
};

#endif // HUMANOIDMODELS_HUMANOID_OPENNIADAPTERNODE_H_
