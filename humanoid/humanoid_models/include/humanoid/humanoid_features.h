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
#include <humanoid_models/HumanoidStateArray.h>

namespace humanoid
{

typedef std::string _JointName;
typedef std::vector<_JointName> _JointNames;
typedef humanoid_models::PoseWithConfidence _PoseWithConfidenceMsg;
typedef humanoid_models::HumanoidJoint _HumanoidJointMsg;
typedef humanoid_models::HumanoidState _HumanoidStateMsg;
typedef humanoid_models::HumanoidStateArray _HumanoidStateArrayMsg;

const static _JointNames JOINT_NAMES_
{
    "head",
    "neck",
    "torso",
    "waist",
    "right_collar",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
    "right_hand",
    "right_finger_tip",
    "left_collar",
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "left_hand",
    "left_finger_tip",
    "right_hip",
    "right_knee",
    "right_ankle",
    "right_foot",
    "left_hip",
    "left_knee",
    "left_ankle",
    "left_foot"
};

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

    const Humanoid & updateJoint( const _HumanoidJointMsg & msg )
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

#endif // HUMANOIDMODELS_HUMANOID_HUMANOIDFEATURES_H_
