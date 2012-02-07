/***************************************************************************
 *  include/humanoid/humanoid_features.h
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

#ifndef HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_
#define HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_

#include <vector>
#include <string>

#include <quickdev/message_array_cache.h>
#include <quickdev/geometry_message_conversions.h>

#include <humanoid_models/HumanoidStateArray.h>
#include <humanoid_models/JointStateArray.h>

namespace humanoid
{

typedef std::string _JointName;
typedef std::vector<_JointName> _JointNames;
typedef std::map<_JointName, std::vector<int> > _JointStateMap;
typedef humanoid_models::PoseWithConfidence _PoseWithConfidenceMsg;
typedef humanoid_models::HumanoidJoint _HumanoidJointMsg;
typedef humanoid_models::HumanoidState _HumanoidStateMsg;
typedef humanoid_models::HumanoidStateArray _HumanoidStateArrayMsg;
typedef sensor_msgs::JointState _JointStateMsg;
typedef humanoid_models::JointStateArray _JointStateArraymsg;

static _JointNames const JOINT_NAMES_
{
    "head",             // 0  : rpy
    "neck",             // 1  : rpy
    "torso",            // 2  : rpy
    "waist",            // 3  : rpy
    "right_collar",     // 4  : nnn
    "right_shoulder",   // 5  : rny
    "right_elbow",      // 6  : rpn
    "right_wrist",      // 7  : rpy
    "right_hand",       // 8  : nnn
    "right_finger_tip", // 9  : nnn
    "left_collar",      // 10 : nnn
    "left_shoulder",    // 11 : rny
    "left_elbow",       // 12 : rpn
    "left_wrist",       // 13 : rpy
    "left_hand",        // 14 : nnn
    "left_finger_tip",  // 15 : nnn
    "right_hip",        // 16 : rpy
    "right_knee",       // 17 : npn
    "right_ankle",      // 18 : rpy
    "right_foot",       // 19 : nnn
    "left_hip",         // 20 : rpy
    "left_knee",        // 21 : npn
    "left_ankle",       // 22 : rpy
    "left_foot"         // 23 : nnn
};

namespace AnatomicalPlanes
{
    static unsigned int const NONE = 0;
    static unsigned int const CORONAL = 1;
    static unsigned int const SAGITTAL = 2;
    static unsigned int const TRANSVERSE = 3;

    static unsigned int const & N = NONE;
    static unsigned int const & C = CORONAL;
    static unsigned int const & S = SAGITTAL;
    static unsigned int const & T = TRANSVERSE;

    static unsigned int const & R = CORONAL;
    static unsigned int const & P = SAGITTAL;
    static unsigned int const & Y = TRANSVERSE;

    static std::vector<std::string> const names { "none", "coronal", "sagittal", "transverse" };
}

static _JointStateMap generateJointStateMap()
{
    // map from [name]{ r, p, y } -> { plane_code, plane_code, plane_code }
    _JointStateMap joint_state_map;

    auto const & n = AnatomicalPlanes::NONE;
    auto const & r = AnatomicalPlanes::CORONAL;
    auto const & p = AnatomicalPlanes::SAGITTAL;
    auto const & y = AnatomicalPlanes::TRANSVERSE;

    joint_state_map[JOINT_NAMES_[0]]  = { r, p, y };
    joint_state_map[JOINT_NAMES_[1]]  = { r, p, y };
    joint_state_map[JOINT_NAMES_[2]]  = { r, p, y };
    joint_state_map[JOINT_NAMES_[3]]  = { r, p, y };
    joint_state_map[JOINT_NAMES_[4]]  = { n, n, n };
    joint_state_map[JOINT_NAMES_[5]]  = { r, n, y };
    joint_state_map[JOINT_NAMES_[6]]  = { r, p, n };
    joint_state_map[JOINT_NAMES_[7]]  = { r, p, y };
    joint_state_map[JOINT_NAMES_[8]]  = { n, n, n };
    joint_state_map[JOINT_NAMES_[9]]  = { n, n, n };
    joint_state_map[JOINT_NAMES_[10]] = { n, n, n };
    joint_state_map[JOINT_NAMES_[11]] = { r, n, y };
    joint_state_map[JOINT_NAMES_[12]] = { r, p, n };
    joint_state_map[JOINT_NAMES_[13]] = { r, p, y };
    joint_state_map[JOINT_NAMES_[14]] = { n, n, n };
    joint_state_map[JOINT_NAMES_[15]] = { n, n, n };
    joint_state_map[JOINT_NAMES_[16]] = { r, p, y };
    joint_state_map[JOINT_NAMES_[17]] = { n, p, n };
    joint_state_map[JOINT_NAMES_[18]] = { r, p, y };
    joint_state_map[JOINT_NAMES_[19]] = { n, n, n };
    joint_state_map[JOINT_NAMES_[20]] = { r, p, y };
    joint_state_map[JOINT_NAMES_[21]] = { n, p, n };
    joint_state_map[JOINT_NAMES_[22]] = { r, p, y };
    joint_state_map[JOINT_NAMES_[23]] = { n, n, n };

    return joint_state_map;
}

static auto getJointStateMap() -> decltype( generateJointStateMap() )
{
    static bool generated = false;
    static decltype( generateJointStateMap() ) joint_state_map;

    if( !generated )
    {
        joint_state_map = generateJointStateMap();
        generated = true;
    }

    return joint_state_map;
}

class Humanoid : public quickdev::NamedMessageArrayCache<_HumanoidJointMsg>, public quickdev::TimedMessageArrayCache<_HumanoidJointMsg>
{
public:
    typedef quickdev::NamedMessageArrayCache<_HumanoidJointMsg> _NamedMessageArrayCache;
    typedef quickdev::TimedMessageArrayCache<_HumanoidJointMsg> _TimedMessageArrayCache;

    struct Header
    {
        ros::Time stamp;
    };

    std::string name;
    Header header;

protected:
    _HumanoidStateMsg joint_array_msg_;

public:
    Humanoid( const _HumanoidStateMsg & msg = _HumanoidStateMsg() )
    :
        _NamedMessageArrayCache(),
        _TimedMessageArrayCache( _NamedMessageArrayCache::getStorage() )
    {
        updateJoints( msg );
    }

    Humanoid & updateJoints( const _HumanoidStateMsg & msg )
    {
        name = msg.name;
        header.stamp = msg.header.stamp;

        _NamedMessageArrayCache::updateMessages( msg.joints );

        return *this;
    }

    void eraseOld( double const & message_lifetime = 1.0 )
    {
        _TimedMessageArrayCache::eraseOld( message_lifetime );
    }

    Humanoid & operator=( const _HumanoidStateMsg & msg )
    {
        return updateJoints( msg );
    }

    const _HumanoidStateMsg & getJointsMessage()
    {
        // triggers re-building of the cached ROS message, if necessary
        joint_array_msg_.joints = _NamedMessageArrayCache::getMessages();
        joint_array_msg_.header.stamp = _TimedMessageArrayCache::getStamp();
        return joint_array_msg_;
    }

    _JointStateMsg getJointStateMsg()
    {
        _JointStateMsg joint_state_msg;

        // ensure we have at most 3 * number_of_joints to work with (since we'll be making a "joint state" for every rotational component of each joint )
        auto const num_joint_states = _NamedMessageArrayCache::size() * 3;

        joint_state_msg.name.reserve( num_joint_states );
        joint_state_msg.position.reserve( num_joint_states );

        joint_state_msg.velocity.resize( num_joint_states );
        joint_state_msg.effort.resize( num_joint_states );

        auto const & joint_state_map = getJointStateMap();

        for( auto joint = _NamedMessageArrayCache::cbegin(); joint != _NamedMessageArrayCache::cend(); ++joint )
        {
            // { [0-3], [0-3], [0-3] }
            auto const & joint_state_entry = joint_state_map.find(joint->name)->second;
            // convert: btVector3 <- btQuaternion <- geometry_msgs::Quaternion
            btVector3 const position_vec = unit::make_unit( unit::convert<btQuaternion>( joint->pose.pose.orientation ) );

            for( auto component_id = joint_state_entry.cbegin(); component_id != joint_state_entry.cend(); ++component_id )
            {
                if( *component_id == 0 ) continue;

                joint_state_msg.name.push_back( joint->name + "_" + AnatomicalPlanes::names[*component_id] );
                joint_state_msg.position.push_back( position_vec.m_floats[*component_id - 1] );
            }
        }

        return joint_state_msg;
    }

    const Humanoid & updateJoint( _HumanoidJointMsg const & msg )
    {
        _NamedMessageArrayCache::updateMessage( msg );

        return *this;
    }

    size_t size() const
    {
        return _NamedMessageArrayCache::size();
    }
};

typedef Humanoid _Humanoid;
typedef std::pair<_Humanoid, _Humanoid> _HumanoidPair;

} // humanoid

//! conversion: _HumanoidJointMsg -> btVector3
DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_HumanoidJointMsg, btVector3, msg, return btVector3( msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ); )
//! conversion: _HumanoidJointMsg -> btQuaternion
DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_HumanoidJointMsg, btQuaternion, msg, return btQuaternion( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ); )
//! conversion: _HumanoidJointMsg -> btTransform
DECLARE_UNIT_CONVERSION_LAMBDA( humanoid::_HumanoidJointMsg, btTransform, msg, return btTransform( btQuaternion( msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ), btVector3( msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z ) ); )

#endif // HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_
