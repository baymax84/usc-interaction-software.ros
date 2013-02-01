/***************************************************************************
 *  include/openni_multitracker/openni_adapter_node.h
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

using humanoid::_Humanoid;
using humanoid::_HumanoidStateArrayMsg;
using humanoid::_HumanoidStateMsg;
using humanoid::_HumanoidJointMsg;
using humanoid::_JointName;
typedef std::map<_JointName, tf::Quaternion> _JointNormMap;
typedef std::map<_JointName, std::vector<double> > _JointErrorMap;

static _JointErrorMap generateJointErrorMap()
{
    _JointErrorMap joint_error_map;

    // joint_error_map["joint"]            = { r, p, y };
    joint_error_map["head"]             = { 0, 0, 0 };
    joint_error_map["neck"]             = { 0, 0, 0 };
    joint_error_map["torso"]            = { 0, 0, 0 };
//    joint_error_map["waist"]            = { 0, 0, 0 };
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

static _JointErrorMap generateJointOffsetMap()
{
    _JointErrorMap joint_offset_map;

    // joint_offset_map["joint"]            = { x, y, z };
    joint_offset_map["head"]             = { 0, 0, 0 };
    joint_offset_map["neck"]             = { 0, 0, 0 };
    joint_offset_map["torso"]            = { 0, 0, 0 };
//    joint_offset_map["waist"]            = { 0, 0, 0 };
    joint_offset_map["right_collar"]     = { 0, 0, 0 };
    joint_offset_map["right_shoulder"]   = { 0, 0, 0 };
    joint_offset_map["right_elbow"]      = { 0, 0, 0 };
    joint_offset_map["right_wrist"]      = { 0, 0, 0 };
    joint_offset_map["right_hand"]       = { 0, 0, 0 };
    joint_offset_map["right_finger_tip"] = { 0, 0, 0 };
    joint_offset_map["left_collar"]      = { 0, 0, 0 };
    joint_offset_map["left_shoulder"]    = { 0, 0, 0 };
    joint_offset_map["left_elbow"]       = { 0, 0, 0 };
    joint_offset_map["left_wrist"]       = { 0, 0, 0 };
    joint_offset_map["left_hand"]        = { 0, 0, 0 };
    joint_offset_map["left_finger_tip"]  = { 0, 0, 0 };
    joint_offset_map["right_hip"]        = { 0, 0, 0 };
    joint_offset_map["right_knee"]       = { 0, 0, 0 };
    joint_offset_map["right_ankle"]      = { 0, 0, 0 };
    joint_offset_map["right_foot"]       = { 0, 0, 0 };
    joint_offset_map["left_hip"]         = { 0, 0, 0 };
    joint_offset_map["left_knee"]        = { 0, 0, 0 };
    joint_offset_map["left_ankle"]       = { 0, 0, 0 };
    joint_offset_map["left_foot"]        = { 0, 0, 0 };

    return joint_offset_map;
}

static _JointNormMap generateJointNormMap()
{
    _JointNormMap joint_norm_map;

    joint_norm_map["head"]             = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["neck"]             = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["torso"]            = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
//    joint_norm_map["waist"]            = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_collar"]     = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_shoulder"]   = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( -180 ) ) );
    joint_norm_map["right_elbow"]      = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 0  ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_wrist"]      = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_hand"]       = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_finger_tip"] = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_collar"]      = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_shoulder"]    = tf::Quaternion( Radian( Degree( 90  ) ), Radian( Degree( 90 ) ), Radian( Degree( -180 ) ) );
    joint_norm_map["left_elbow"]       = tf::Quaternion( Radian( Degree( 90  ) ), Radian( Degree( 0  ) ), Radian( Degree( -180 ) ) );
    joint_norm_map["left_wrist"]       = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_hand"]        = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_finger_tip"]  = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["right_hip"]        = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["right_knee"]       = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["right_ankle"]      = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["right_foot"]       = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );
    joint_norm_map["left_hip"]         = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["left_knee"]        = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["left_ankle"]       = tf::Quaternion( Radian( Degree( -90 ) ), Radian( Degree( 90 ) ), Radian( Degree( 90   ) ) );
    joint_norm_map["left_foot"]        = tf::Quaternion( Radian( Degree( 0   ) ), Radian( Degree( 90 ) ), Radian( Degree( -90  ) ) );

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

static auto getJointOffsetMap() -> decltype( generateJointOffsetMap() ) const &
{
    static auto const & joint_offset_map = generateJointOffsetMap();

    return joint_offset_map;
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

            _HumanoidStateMsg humanoid_state_msg;
            humanoid_state_msg.header.stamp = now;
            humanoid_state_msg.header.frame_id = "openni_depth_frame";

            humanoid_state_msg.name = user_state->name;

            for( auto joint_name_it = humanoid::JOINT_NAMES_.begin(); joint_name_it != humanoid::JOINT_NAMES_.end(); ++joint_name_it )
            {

                auto const & joint_name = *joint_name_it;
                if( joint_name == "pelvis" ) continue;

                auto const parent_joint_name = joint_dependency_map.find( joint_name )->second;

                auto const joint_frame_name( "/" + user_state->name + "/" + joint_name );
                //auto const parent_joint_frame_name( "/" + user_state->name + "/" + parent_joint_name );

                if( transformExists( "/openni_depth_frame", joint_frame_name ) )
                {
                    auto const sensor_to_current_joint_tf( lookupTransform( "/openni_depth_frame", joint_frame_name ) );
                    //auto const parent_transform( lookupTransform( parent_joint_frame_name, joint_frame_name ) );

                    auto const norm_rotation = joint_norm_map.find( joint_name )->second;
                    tf::Transform const normalized_transform( sensor_to_current_joint_tf.getRotation() * norm_rotation, sensor_to_current_joint_tf.getOrigin() );

                    _HumanoidJointMsg joint_msg;

                    joint_msg.header.stamp = now;
                    joint_msg.header.frame_id = "openni_depth_frame";

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

                    humanoid_state_msg.joints.push_back( joint_msg );
                    _TfTranceiverPolicy::publishTransform( normalized_transform, "/openni_depth_frame", joint_frame_name + "_norm" );
                }
            }

            // calculate additional frames (with covariance) given some minimum info (eyes, ears, nose, mouth, etc)
            humanoid_state_msg = _Humanoid::estimateExtraJoints( humanoid_state_msg );

            for( auto joint_msg_it = humanoid_state_msg.joints.cbegin(); joint_msg_it != humanoid_state_msg.joints.cend(); ++joint_msg_it )
            {
                if( joint_msg_it->name == "pelvis" )
                {
                    auto const & joint_msg = *joint_msg_it;
                    _TfTranceiverPolicy::publishTransform( unit::convert<tf::Transform>( joint_msg ), "/openni_depth_frame", "/" + user_state->name + "/" + joint_msg.name );
                }
            }

            // calculate base link frame
            tf::Transform const world_to_sensor_tf = _TfTranceiverPolicy::tryLookupTransform( "/world", "/openni_depth_frame" );
            tf::Transform const sensor_to_base_link_tf = _Humanoid::calculateBaseLinkTransform( humanoid_state_msg, world_to_sensor_tf );

            // publish base link tf frame
            _TfTranceiverPolicy::publishTransform( sensor_to_base_link_tf, "/openni_depth_frame", "/" + user_state->name + "/base_link" );

            // create base link joint
            _HumanoidJointMsg base_link_joint;
            base_link_joint.header.stamp = now;
            base_link_joint.header.frame_id = "openni_depth_frame";
            base_link_joint.name = "base_link";
            base_link_joint.pose.pose = unit::implicit_convert( sensor_to_base_link_tf );

            // append base link joint to list of joints
            humanoid_state_msg.joints.push_back( base_link_joint );

            state_array_msg.states.push_back( humanoid_state_msg );
        }

        if( state_array_msg.states.size() > 0 ) multi_pub_.publish( "humanoid_states", quickdev::make_const_shared( state_array_msg ) );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( userStatesCB, _UserStateArrayMsg )
    {
        _UserStatesCBTimer::update();

        QUICKDEV_TRY_UPDATE_CACHE( user_states_cache_, msg );
        QUICKDEV_TRY_LOCK_OR_WARN( user_states_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );
    }
};

#endif // HUMANOIDMODELS_HUMANOID_OPENNIADAPTERNODE_H_
