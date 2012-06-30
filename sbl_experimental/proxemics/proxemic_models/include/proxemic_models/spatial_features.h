/***************************************************************************
 *  include/proxemic_models/spatial_features.h
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

#ifndef PROXEMICMODELS_SPATIALFEATURES_H_
#define PROXEMICMODELS_SPATIALFEATURES_H_

// conversions
#include <quickdev/geometry_message_conversions.h>
#include <quickdev/numeric_unit_conversions.h>

// objects
#include <humanoid/humanoid_features.h>
#include <LinearMath/btVector3.h>

#include <proxemic_models/SpatialFeatureArray.h>

namespace proxemics
{
    typedef proxemic_models::SpatialFeature _SpatialFeatureMsg;
    typedef proxemic_models::SpatialFeatureArray _SpatialFeatureArrayMsg;

namespace spatial
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;

    //! 3D distance along ray from joint1 to joint2
    static double getDistanceTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btVector3 joint1_pos = unit::make_unit( joint1 );
        btVector3 joint2_pos = unit::make_unit( joint2 );

        return joint1_pos.distance( joint2_pos );
    }

    //! 3D per-axis distance between joint1 and joint2
    static btVector3 getComponentDistanceTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btVector3 joint1_pos = unit::make_unit( joint1 );
        btVector3 joint2_pos = unit::make_unit( joint2 );

        return joint2_pos - joint1_pos;
    }

    //! Midpoint between joint1 and joint2
    static btVector3 getMidpoint( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btVector3 joint1_pos = unit::make_unit( joint1 );
        btVector3 joint2_pos = unit::make_unit( joint2 );

        return 0.5 * ( joint1_pos + joint2_pos );
    }

    //! Transform from joint1 to joint2 (order matters)
    static btTransform getTransformTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btTransform joint1_tf = unit::make_unit( joint1 );
        btTransform joint2_tf = unit::make_unit( joint2 );

        return joint1_tf.inverse() * joint2_tf;
    }

    //! Angle from joint1 to joint2 (order matters)
    static btQuaternion getAngleTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        return getTransformTo( joint1, joint2 ).getRotation();
    }

    //! Angle from joint1 to joint2 (YPR)
    static btVector3 getAngleToYPR( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        return unit::convert<btVector3>( getAngleTo( joint1, joint2 ) );
    }
} // spatial

class SpatialFeatureRecognizer
{
public:
    typedef humanoid::_Humanoid _Humanoid;

protected:
    _Humanoid humanoid_;

public:
    SpatialFeatureRecognizer( _Humanoid const & humanoid = _Humanoid() )
    :
        humanoid_( humanoid )
    {
        //
    }

    btVector3 getOrigin() const
    {
        return unit::convert<btVector3>( humanoid_["torso"] );
    }

    double getRotation() const
    {
        return unit::convert<btVector3>( unit::convert<btQuaternion>( humanoid_["torso"] ) ).getZ();
    }

    double getDistanceTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];
        return proxemics::spatial::getDistanceTo( from_joint, to_joint );
    }

    btVector3 getComponentDistanceTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];
        return proxemics::spatial::getComponentDistanceTo( from_joint, to_joint );
    }

    btVector3 getMidpoint( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];
        return proxemics::spatial::getMidpoint( from_joint, to_joint );
    }

    btTransform getTransformTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];
        return proxemics::spatial::getTransformTo( from_joint, to_joint );
    }

    btQuaternion getAngleTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];
        return proxemics::spatial::getAngleTo( from_joint, to_joint );
    }

    btVector3 getAngleToYPR( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];
        return proxemics::spatial::getAngleToYPR( from_joint, to_joint );
    }

    _SpatialFeatureMsg createMessage( SpatialFeatureRecognizer const & other ) const
    {
        _SpatialFeatureMsg spatial_feature_msg;
        spatial_feature_msg.header.stamp = ros::Time::now();
        spatial_feature_msg.observer_name = humanoid_.name;
        spatial_feature_msg.target_name = other.humanoid_.name;
        spatial_feature_msg.distance = getComponentDistanceTo( other ).getX();
        spatial_feature_msg.orientation = getAngleToYPR( other ).getZ();
        return spatial_feature_msg;
    }
};

} // proxemics

#endif // PROXEMICMODELS_SPATIALFEATURES_H_
