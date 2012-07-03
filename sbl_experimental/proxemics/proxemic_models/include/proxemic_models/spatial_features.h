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
#include <quickdev/feature_with_covariance.h>

// msgs
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

template<unsigned int __Dim__, class __Data = double>
class Covariance : public std::array<__Data, __Dim__ * __Dim__>
{
public:
    typedef std::array<__Data, __Dim__ * __Dim__> _Storage;

    // empty
    Covariance()
    {
        //
    }

    // initializer list
    Covariance( std::initializer_list<__Data> values )
    {
        std::copy( values.cbegin(), values.cend(), this->begin() );
    }

    // args == __Dim__ * __Dim__
    template
    <
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) == __Dim__ * __Dim__), int>::type = 0
    >
    Covariance( __Args&&... args )
    :
        _Storage{{ std::forward<__Args>( args )... }}
    {
        //
    }

    Covariance( __Data const & value )
    {
        this->fill( value );
    }
};

class SpatialFeatureRecognizer
{
public:
    typedef humanoid::_Humanoid _Humanoid;

    _Humanoid humanoid_;

public:
    SpatialFeatureRecognizer(){}

    SpatialFeatureRecognizer( _Humanoid const & humanoid = _Humanoid() )
    :
        humanoid_( humanoid )
    {
        //
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getOrigin( std::string const & joint_name = "torso" ) const
    {
        auto const & joint = humanoid_[joint_name];

        quickdev::FeatureWithCovariance<btVector3, 3> result( unit::convert<btVector3>( joint ) );

        result.getCovariance()( 0, 0 ) = joint.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = joint.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = joint.pose.covariance[2 * 6 + 2];

        return result;
    }

    quickdev::FeatureWithCovariance<double, 1> getRotation( std::string const & joint_name = "torso" ) const
    {
        auto const & joint = humanoid_[joint_name];

        quickdev::FeatureWithCovariance<double, 1> result( unit::convert<btVector3>( unit::convert<btQuaternion>( humanoid_[joint_name] ) ).getZ() );

        result.getCovariance()( 0, 0 ) = joint.pose.covariance[5 * 6 + 5];

        return result;
    }

    quickdev::FeatureWithCovariance<double, 1> getDistanceTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        quickdev::FeatureWithCovariance<double, 1> result( proxemics::spatial::getDistanceTo( from_joint, to_joint ) );

        result.getCovariance()( 0, 0 ) = 0; // unkown how to perform covariance calculation for this operation

        return result;
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getComponentDistanceTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        quickdev::FeatureWithCovariance<btVector3, 3> result( proxemics::spatial::getComponentDistanceTo( from_joint, to_joint ) );

        result.getCovariance()( 0, 0 ) = from_joint.pose.covariance[0 * 6 + 0] + to_joint.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = from_joint.pose.covariance[1 * 6 + 1] + to_joint.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = from_joint.pose.covariance[2 * 6 + 2] + to_joint.pose.covariance[2 * 6 + 2];

        return result;
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getMidpoint( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        quickdev::FeatureWithCovariance<btVector3, 3> result( proxemics::spatial::getMidpoint( from_joint, to_joint ) );

        result.getCovariance()( 0, 0 ) = from_joint.pose.covariance[0 * 6 + 0] + to_joint.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = from_joint.pose.covariance[1 * 6 + 1] + to_joint.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = from_joint.pose.covariance[2 * 6 + 2] + to_joint.pose.covariance[2 * 6 + 2];

        return result;
    }

    quickdev::FeatureWithCovariance<btTransform, 6> getTransformTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        quickdev::FeatureWithCovariance<btTransform, 6> result( proxemics::spatial::getTransformTo( from_joint, to_joint ) );

        result.getCovariance()( 0, 0 ) = from_joint.pose.covariance[0 * 6 + 0] + to_joint.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = from_joint.pose.covariance[1 * 6 + 1] + to_joint.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = from_joint.pose.covariance[2 * 6 + 2] + to_joint.pose.covariance[2 * 6 + 2];
        result.getCovariance()( 3, 3 ) = from_joint.pose.covariance[3 * 6 + 3] + to_joint.pose.covariance[3 * 6 + 3];
        result.getCovariance()( 4, 4 ) = from_joint.pose.covariance[4 * 6 + 4] + to_joint.pose.covariance[4 * 6 + 4];
        result.getCovariance()( 5, 5 ) = from_joint.pose.covariance[5 * 6 + 5] + to_joint.pose.covariance[5 * 6 + 5];

        return result;
    }

    quickdev::FeatureWithCovariance<btQuaternion, 3> getAngleTo( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        quickdev::FeatureWithCovariance<btQuaternion, 3> result( proxemics::spatial::getAngleTo( from_joint, to_joint ) );

        result.getCovariance()( 3, 3 ) = from_joint.pose.covariance[3 * 6 + 3] + to_joint.pose.covariance[3 * 6 + 3];
        result.getCovariance()( 4, 4 ) = from_joint.pose.covariance[4 * 6 + 4] + to_joint.pose.covariance[4 * 6 + 4];
        result.getCovariance()( 5, 5 ) = from_joint.pose.covariance[5 * 6 + 5] + to_joint.pose.covariance[5 * 6 + 5];

        return result;
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getAngleToYPR( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        quickdev::FeatureWithCovariance<btVector3, 3> result( proxemics::spatial::getAngleToYPR( from_joint, to_joint ) );

        result.getCovariance()( 3, 3 ) = from_joint.pose.covariance[3 * 6 + 3] + to_joint.pose.covariance[3 * 6 + 3];
        result.getCovariance()( 4, 4 ) = from_joint.pose.covariance[4 * 6 + 4] + to_joint.pose.covariance[4 * 6 + 4];
        result.getCovariance()( 5, 5 ) = from_joint.pose.covariance[5 * 6 + 5] + to_joint.pose.covariance[5 * 6 + 5];

        return result;
    }

    _SpatialFeatureMsg createMessage( SpatialFeatureRecognizer const & other, std::string const & from_joint_name = "torso", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        _SpatialFeatureMsg spatial_feature_msg;
        spatial_feature_msg.header.stamp = ros::Time::now();
        spatial_feature_msg.observer_name = humanoid_.name;
        spatial_feature_msg.target_name = other.humanoid_.name;
        spatial_feature_msg.covariance.assign( 0.0 );

        auto const & distance = getComponentDistanceTo( other, from_joint_name, to_joint_name );

        spatial_feature_msg.pose.x = distance.getFeature().getX();
        spatial_feature_msg.pose.y = distance.getFeature().getY();

        spatial_feature_msg.covariance[0 * 3 + 0] = distance.getCovariance()( 0, 0 );
        spatial_feature_msg.covariance[1 * 3 + 1] = distance.getCovariance()( 1, 1 );

        // the angle here is the amount the X-Z plane of the target is turned away from the X-Z plane of the observer (ie does not change with translation)
        // so if the target is facing north and the observer is facing north, this angle is 180 degrees
        // if the target is facing north and the observer is facing south, this angle is 0 degrees
        // if the target is facing north and the observer is facing east, this angle is -90 degrees
        // if the target is facing west and the observer is facing south, this angle is 90 degrees
        auto const & rotation1 = getRotation();
        auto const & rotation2 = other.getRotation();

        spatial_feature_msg.pose.theta = ( rotation2.getFeature() - rotation1.getFeature() ) + M_PI;

        spatial_feature_msg.covariance[2 * 3 + 2] = rotation1.getCovariance()( 0, 0 ) + rotation2.getCovariance()( 0, 0 );
        return spatial_feature_msg;
    }
};

} // proxemics

#endif // PROXEMICMODELS_SPATIALFEATURES_H_
