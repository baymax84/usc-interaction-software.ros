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

// objects
#include <humanoid/humanoid_features.h>
#include <humanoid/soft_classification.h>

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
    static quickdev::FeatureWithCovariance<double, 1> getDistanceTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btVector3 joint1_pos = unit::make_unit( joint1 );
        btVector3 joint2_pos = unit::make_unit( joint2 );

        quickdev::FeatureWithCovariance<double, 1> result( joint1_pos.distance( joint2_pos ) );

        // here we assume that all linear components have identical static variance
        result.getCovariance()( 0, 0 ) = joint1.pose.covariance[0 * 6 + 0] + joint2.pose.covariance[0 * 6 + 0];

        return result;
    }

    //! 2D (x-y) distance along ray from joint1 to joint2
    static quickdev::FeatureWithCovariance<double, 1> getDistance2DTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btVector3 joint1_pos = unit::make_unit( joint1 );
        btVector3 joint2_pos = unit::make_unit( joint2 );

        joint1_pos.setZ( 0 );
        joint2_pos.setZ( 0 );

        quickdev::FeatureWithCovariance<double, 1> result( joint1_pos.distance( joint2_pos ) );

//        PRINT_INFO( "variance1: %f; variance2: %f", joint1.pose.covariance[0 * 6 + 0], joint2.pose.covariance[0 * 6 + 0] );

        // here we assume that all linear components have identical static variance
        result.getCovariance()( 0, 0 ) = joint1.pose.covariance[0 * 6 + 0] + joint2.pose.covariance[0 * 6 + 0];

        return result;
    }

    //! 3D per-axis distance between joint1 and joint2
    static quickdev::FeatureWithCovariance<btVector3, 3> getComponentDistanceTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btTransform const world_to_joint1_tf = unit::make_unit( joint1 );
        btTransform const world_to_joint2_tf = unit::make_unit( joint2 );

        btTransform const joint1_to_joint2_tf = world_to_joint1_tf.inverse() * world_to_joint2_tf;

        quickdev::FeatureWithCovariance<btVector3, 3> result( joint1_to_joint2_tf.getOrigin() );

        result.getCovariance()( 0, 0 ) = joint1.pose.covariance[0 * 6 + 0] + joint2.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = joint1.pose.covariance[1 * 6 + 1] + joint2.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = joint1.pose.covariance[2 * 6 + 2] + joint2.pose.covariance[2 * 6 + 2];

        return result;
    }

    //! Midpoint between joint1 and joint2
    static quickdev::FeatureWithCovariance<btVector3, 3> getMidpoint( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btVector3 joint1_pos = unit::make_unit( joint1 );
        btVector3 joint2_pos = unit::make_unit( joint2 );

        quickdev::FeatureWithCovariance<btVector3, 3> result( 0.5 * ( joint1_pos + joint2_pos ) );

        result.getCovariance()( 0, 0 ) = joint1.pose.covariance[0 * 6 + 0] + joint2.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = joint1.pose.covariance[1 * 6 + 1] + joint2.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = joint1.pose.covariance[2 * 6 + 2] + joint2.pose.covariance[2 * 6 + 2];

        return result;
    }

    //! Transform from joint1 to joint2 (order matters)
    static quickdev::FeatureWithCovariance<btTransform, 6> getTransformTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        btTransform joint1_tf = unit::make_unit( joint1 );
        btTransform joint2_tf = unit::make_unit( joint2 );

        quickdev::FeatureWithCovariance<btTransform, 6> result( joint1_tf.inverse() * joint2_tf );

        result.getCovariance()( 0, 0 ) = joint1.pose.covariance[0 * 6 + 0] + joint2.pose.covariance[0 * 6 + 0];
        result.getCovariance()( 1, 1 ) = joint1.pose.covariance[1 * 6 + 1] + joint2.pose.covariance[1 * 6 + 1];
        result.getCovariance()( 2, 2 ) = joint1.pose.covariance[2 * 6 + 2] + joint2.pose.covariance[2 * 6 + 2];
        result.getCovariance()( 3, 3 ) = joint1.pose.covariance[3 * 6 + 3] + joint2.pose.covariance[3 * 6 + 3];
        result.getCovariance()( 4, 4 ) = joint1.pose.covariance[4 * 6 + 4] + joint2.pose.covariance[4 * 6 + 4];
        result.getCovariance()( 5, 5 ) = joint1.pose.covariance[5 * 6 + 5] + joint2.pose.covariance[5 * 6 + 5];

        return result;
    }

    //! Angle from joint1 to joint2 (order matters)
    static quickdev::FeatureWithCovariance<btQuaternion, 3> getAngleTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        auto const transform = getTransformTo( joint1, joint2 );
        quickdev::FeatureWithCovariance<btQuaternion, 3> result( transform.getFeature().getRotation() );

        result.getCovariance()( 0, 0 ) = transform.getCovariance()( 3, 3 );
        result.getCovariance()( 1, 1 ) = transform.getCovariance()( 4, 4 );
        result.getCovariance()( 2, 2 ) = transform.getCovariance()( 5, 5 );

        return result;
    }

    //! Angle from joint1 to joint2 (YPR)
    static quickdev::FeatureWithCovariance<btVector3, 3> getAngleToYPR( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        auto const angle = getAngleTo( joint1, joint2 );
        quickdev::FeatureWithCovariance<btVector3, 3> result( unit::convert<btVector3>( angle.getFeature() ) );

        result.getCovariance() = angle.getCovariance();

        return result;
    }

    //! Angle from joint1 to joint2 (2D, yaw)
    static quickdev::FeatureWithCovariance<double, 1> getAngle2DTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        auto const component_distance = getComponentDistanceTo( joint1, joint2 );
        quickdev::FeatureWithCovariance<double, 1> result( atan2( component_distance.getFeature().getY(), component_distance.getFeature().getX() ) );

        result.getCovariance()( 0, 0 ) = component_distance.getCovariance()( 0, 0 ) + component_distance.getCovariance()( 1, 1 );

        return result;
    }
} // spatial

class SpatialFeatureRecognizer : public humanoid::HumanoidFeatureRecognizer
{
public:
    typedef humanoid::HumanoidFeatureRecognizer _HumanoidFeatureRecognizer;
    typedef humanoid::Humanoid _Humanoid;

public:
    SpatialFeatureRecognizer( _Humanoid const & humanoid = _Humanoid() )
    :
        humanoid::HumanoidFeatureRecognizer( humanoid )
    {
        //
    }

    quickdev::FeatureWithCovariance<double, 1> getDistanceTo( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getDistanceTo( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<double, 1> getDistance2DTo( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

//        PRINT_INFO( "from_joint variance: %f; to_joint variance: %f", from_joint.pose.covariance[0 * 6 + 0], to_joint.pose.covariance[0 * 6 + 0] );

        return proxemics::spatial::getDistance2DTo( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getComponentDistanceTo( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getComponentDistanceTo( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getMidpoint( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getMidpoint( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<btTransform, 6> getTransformTo( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getTransformTo( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<btQuaternion, 3> getAngleTo( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getAngleTo( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<btVector3, 3> getAngleToYPR( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getAngleToYPR( from_joint, to_joint );
    }

    quickdev::FeatureWithCovariance<double, 1> getAngle2DTo( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        return proxemics::spatial::getAngle2DTo( from_joint, to_joint );
    }

    _SpatialFeatureMsg createMessage( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" ) const
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        _SpatialFeatureMsg spatial_feature_msg;
        spatial_feature_msg.header.stamp = ros::Time::now();
        spatial_feature_msg.observer_name = humanoid_.name;
        spatial_feature_msg.target_name = other.humanoid_.name;
        spatial_feature_msg.covariance.assign( 0.0 );

        auto const & distance = getComponentDistanceTo( other, from_joint_name, to_joint_name );

        // note: insert additional variance in x-y distance to target due to variance in theta of observer

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
