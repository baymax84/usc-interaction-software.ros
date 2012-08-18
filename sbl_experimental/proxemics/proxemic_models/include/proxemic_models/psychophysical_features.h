/***************************************************************************
 *  include/proxemic_models/psychophysical_features.h
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

#ifndef PROXEMICMODELS_PSYCHOPHYSICALFEATURES_H_
#define PROXEMICMODELS_PSYCHOPHYSICALFEATURES_H_

#include <proxemic_models/spatial_features.h>
#include <angles/angles.h>
#include <proxemic_models/PsychophysicalFeatureArray.h>
#include <quickdev/param_reader.h>
#include <humanoid/soft_classification.h>
#include <quickdev/cdf_wrapped_normal.h>

namespace proxemics
{
    typedef proxemic_models::PsychophysicalFeature _PsychophysicalFeatureMsg;
    typedef proxemic_models::PsychophysicalFeatureArray _PsychophysicalFeatureArrayMsg;
    typedef proxemic_models::PsychophysicalDimension _PsychophysicalDimensionMsg;

namespace psychophysical
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;
    using humanoid::_SoftClassificationSet;

    // proxemic sex codes
    enum SexCode
    {
        SEX_FEMALE = -1,  // SEX_FEMALE      = -1
        SEX_UNSPECIFIED,  // SEX_UNSPECIFIED =  0
        SEX_MALE          // SEX_MALE        =  1
    };

    // proxemic posture codes
    enum PostureCode
    {
        POSTURE_PRONE = 0,  // POSTURE_PRONE    = 0
        POSTURE_SITTING,    // POSTURE_SITTING  = 1
        POSTURE_STANDING    // POSTURE_STANDING = 2
    };

    // proxemic sociofugal-sociopetal (SFP) axis codes
    enum SfpAxisCode
    {
        SFP_AXIS_0 = 0,  // SFP_AXIS_0 = 0
        SFP_AXIS_1,      // SFP_AXIS_1 = 1
        SFP_AXIS_2,      // SFP_AXIS_2 = 2
        SFP_AXIS_3,      // SFP_AXIS_3 = 3
        SFP_AXIS_4,      // SFP_AXIS_4 = 4
        SFP_AXIS_5,      // SFP_AXIS_5 = 5
        SFP_AXIS_6,      // SFP_AXIS_6 = 6
        SFP_AXIS_7,      // SFP_AXIS_7 = 7
        SFP_AXIS_8       // SFP_AXIS_8 = 8
    };

    // proxemic kinesthetic codes
    enum KinestheticCode
    {
        KINESTHETIC_BODY = 0,           // KINESTHETIC_BODY               = 1
        KINESTHETIC_BODY_OUTSIDE,       // KINESTHETIC_BODY_OUTSIDE       = 2
        KINESTHETIC_LOWER_ARM,          // KINESTHETIC_LOWER_ARM          = 3
        KINESTHETIC_LOWER_ARM_OUTSIDE,  // KINESTHETIC_LOWER_ARM_OUTSIDE  = 4
        KINESTHETIC_WHOLE_ARM,          // KINESTHETIC_WHOLE_ARM          = 5
        KINESTHETIC_WHOLE_ARM_OUTSIDE,  // KINESTHETIC_WHOLE_ARM_OUTSIDE  = 6
        KINESTHETIC_FULL_REACH,         // KINESTHETIC_FULL_REACH         = 7
        KINESTHETIC_FULL_REACH_OUTSIDE  // KINESTHETIC_FULL_REACH_OUTSIDE = 8
    };

    // proxemic touch codes
    enum TouchCode
    {
        TOUCH_CARESSING_HOLDING = 0,  // TOUCH_CARESSING_HOLDING = 0
        TOUCH_CARESSING,              // TOUCH_CARESSING         = 1
        TOUCH_HOLDING_EXTENDED,       // TOUCH_HOLDING_EXTENDED  = 2
        TOUCH_HOLDING,                // TOUCH_HOLDING           = 3
        TOUCH_SPOT,                   // TOUCH_SPOT              = 4
        TOUCH_ACCIDENTAL,             // TOUCH_ACCIDENTAL        = 5
        TOUCH_NONE                    // TOUCH_NONE              = 6
    };

    // proxemic visual codes (retinal combinations)
    enum VisualCode
    {
        VISUAL_FOVEAL = 0,  // VISUAL_FOVEAL     = 0
        VISUAL_MACULAR,     // VISUAL_MACULAR    = 1
        VISUAL_SCANNING,    // VISUAL_SCANNING   = 2
        VISUAL_PERIPHERAL,  // VISUAL_PERIPHERAL = 3
        VISUAL_NONE   = 8   // VISUAL_NONE       = 8
    };

    // proxemic thermal codes
    enum ThermalCode
    {
        THERMAL_CONDUCTED = 1,  // THERMAL_CONDUCTED = 1
        THERMAL_RADIANT,        // THERMAL_RADIANT   = 2
        THERMAL_PROBABLY,       // THERMAL_PROBABLY  = 3
        THERMAL_NONE = 8        // THERMAL_NONE      = 8
    };

    // proxemic olfaction codes
    enum OlfactionCode
    {
        OLFACTION_BODY_ODOR_DIFF = 1,  // OLFACTION_BODY_ODOR_DIFF   = 1
        OLFACTION_BODY_ODOR_UNDIFF,    // OLFACTION_BODY_ODOR_UNDIFF = 2
        OLFACTION_BREATH,              // OLFACTION_BREATH           = 3
        OLFACTION_PROBABLY,            // OLFACTION_PROBABLY         = 4
        OLFACTION_NONE = 8             // OLFACTION_NONE             = 8
    };

    // proxemic voice loudness codes
    enum VoiceLoudnessCode
    {
        VOICE_LOUDNESS_SILENT = 0,   // VOICE_LOUDNESS_SILENT      = 0
        VOICE_LOUDNESS_VERY_SOFT,    // VOICE_LOUDNESS_VERY_SOFT   = 1
        VOICE_LOUDNESS_SOFT,         // VOICE_LOUDNESS_SOFT        = 2
        VOICE_LOUDNESS_NORMAL,       // VOICE_LOUDNESS_NORMAL      = 3
        VOICE_LOUDNESS_NORMAL_PLUS,  // VOICE_LOUDNESS_NORMAL_PLUS = 4
        VOICE_LOUDNESS_LOUD,         // VOICE_LOUDNESS_LOUD        = 5
        VOICE_LOUDNESS_VERY_LOUD,    // VOICE_LOUDNESS_VERY_LOUD   = 6
    };

    // proxemic distances
    enum DistanceCode
    {
        DISTANCE_INTIMATE_CLOSE = 0,  // DISTANCE_INTIMATE_CLOSE = 0
        DISTANCE_INTIMATE_FAR,        // DISTANCE_INTIMATE_FAR   = 1
        DISTANCE_PERSONAL_CLOSE,      // DISTANCE_PERSONAL_CLOSE = 2
        DISTANCE_PERSONAL_FAR,        // DISTANCE_PERSONAL_FAR   = 3
        DISTANCE_SOCIAL_CLOSE,        // DISTANCE_SOCIAL_CLOSE   = 4
        DISTANCE_SOCIAL_FAR,          // DISTANCE_SOCIAL_FAR     = 5
        DISTANCE_PUBLIC_CLOSE,        // DISTANCE_PUBLIC_CLOSE   = 6
        DISTANCE_PUBLIC_FAR           // DISTANCE_PUBLIC_FAR     = 7
    };

    typedef int _ProxemicCode;

    // =========================================================================================================================================
    static _SoftClassificationSet getSexClassification()
    {
        _SoftClassificationSet result;

        result.insert( SoftClassification( SEX_MALE, 1.0, "male" ) );

        return result;
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getPosturalClassification()
    {
        _SoftClassificationSet result;

        result.insert( SoftClassification( POSTURE_STANDING, 1.0, "standing" ) );

        return result;
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getSFPClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & params )
    {
        auto & intervals = params["intervals"];
        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params, "offset", 0 );

        // note: angle from person 2 to person 1 used here because we're trying to find angular displacement from 0 -> 360 between the two people
        auto const angle = proxemics::spatial::getAngle2DTo( joint2, joint1 );
        double const std_dev = sqrt( angle.getCovariance()( 0, 0 ) );
        double const mean = fabs( angle.getFeature() + M_PI ) + Radian( Degree( offset ) );

        _SoftClassificationSet result;

        auto begin = &intervals[0];
        auto end = &intervals[0] + intervals.size();

        double interval_0_8_probability;

        for( XmlRpc::XmlRpcValue * interval_it = begin; interval_it != end; ++interval_it )
        {
            auto & interval = *interval_it;

            auto const interval_id = quickdev::ParamReader::getXmlRpcValue<int>( interval, "id" );
            auto const interval_min = Radian( Degree( quickdev::ParamReader::getXmlRpcValue<double>( interval, "min" ) ) );
            auto const interval_max = Radian( Degree( quickdev::ParamReader::getXmlRpcValue<double>( interval, "max" ) ) );
            auto const interval_name = quickdev::ParamReader::getXmlRpcValue<std::string>( interval, "name" );

            //PRINT_INFO( "evaluating interval %s (%i); u: %f, s: %f; [%f, %f]", interval_name.c_str(), interval_id, mean, std_dev, interval_min, interval_max );

            double const interval_value = cdf_wrapped_normal( mean, std_dev, interval_max ) - cdf_wrapped_normal( mean, std_dev, interval_min );

            if( interval_id == 0 || interval_id == 8 ) interval_0_8_probability += interval_value;
            else result.insert( SoftClassification( interval_id, interval_value, interval_name ) );
        }
        result.insert( SoftClassification( quickdev::ParamReader::getXmlRpcValue<int>( intervals[0], "id" ), interval_0_8_probability, quickdev::ParamReader::getXmlRpcValue<std::string>( intervals[0], "name" ) ) );
        result.insert( SoftClassification( quickdev::ParamReader::getXmlRpcValue<int>( intervals[8], "id" ), interval_0_8_probability, quickdev::ParamReader::getXmlRpcValue<std::string>( intervals[8], "name" ) ) );

        return result;
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getTouchClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & params )
    {
        auto & intervals = params["intervals"];
        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params, "offset", 0 );

        auto const distance = proxemics::spatial::getDistance2DTo( joint1, joint2 );
        double const std_dev = sqrt( distance.getCovariance()( 0, 0 ) );
        double const mean = distance + offset;

        return SoftClassification::sampleIntervals( &intervals[0], &intervals[0] + intervals.size(), mean, std_dev );
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getVisualClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & params )
    {
        auto & intervals = params["intervals"];
        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params, "offset", 0 );

//        quickdev::start_stream_indented( "calculating 2d angle from ", joint1.name, " to ", joint2.name );
        auto const angle = proxemics::spatial::getAngle2DTo( joint1, joint2 );
//        quickdev::end_stream_indented();
//        std::cout << "angle: " << angle.getFeature() << std::endl;
        double const std_dev = sqrt( angle.getCovariance()( 0, 0 ) );
        double const mean = angle.getFeature() + Radian( Degree( offset ) );

        //return SoftClassification::sampleIntervals( &intervals[0], &intervals[0] + intervals.size(), mean, std_dev );

        _SoftClassificationSet result;

        auto begin = &intervals[0];
        auto end = &intervals[0] + intervals.size();

        double last_interval_value = 0;

        for( XmlRpc::XmlRpcValue * interval_it = begin; interval_it != end; ++interval_it )
        {
            auto & interval = *interval_it;

            auto const interval_id = quickdev::ParamReader::getXmlRpcValue<int>( interval, "id" );
            auto const interval_min = Radian( Degree( quickdev::ParamReader::getXmlRpcValue<double>( interval, "min" ) ) );
            auto const interval_max = Radian( Degree( quickdev::ParamReader::getXmlRpcValue<double>( interval, "max" ) ) );
            auto const interval_name = quickdev::ParamReader::getXmlRpcValue<std::string>( interval, "name" );

            //PRINT_INFO( "evaluating interval %s (%i); u: %f, s: %f; [%f, %f]", interval_name.c_str(), interval_id, mean, std_dev, interval_min, interval_max );

            double const interval_value = cdf_wrapped_normal( mean, std_dev, interval_max ) - cdf_wrapped_normal( mean, std_dev, -interval_max );

            result.insert( SoftClassification( interval_id, interval_value - last_interval_value, interval_name ) );

            last_interval_value = interval_value;
        }

        return result;
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getThermalClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & params )
    {
        auto & intervals = params["intervals"];
        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params, "offset", 0 );

        auto const distance = proxemics::spatial::getDistance2DTo( joint1, joint2 );
        double const std_dev = sqrt( distance.getCovariance()( 0, 0 ) );
        double const mean = distance + offset;

        return SoftClassification::sampleIntervals( &intervals[0], &intervals[0] + intervals.size(), mean, std_dev );
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getOlfactionClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & params )
    {
        auto & intervals = params["intervals"];
        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params, "offset", 0 );

        auto const distance = proxemics::spatial::getDistance2DTo( joint1, joint2 );
        double const std_dev = sqrt( distance.getCovariance()( 0, 0 ) );
        double const mean = distance + offset;

        return SoftClassification::sampleIntervals( &intervals[0], &intervals[0] + intervals.size(), mean, std_dev );
    }

    // =========================================================================================================================================
    static _SoftClassificationSet getVoiceLoudnessClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & params )
    {
        auto & intervals = params["intervals"];
        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params, "offset", 0 );

        auto const distance = proxemics::spatial::getDistance2DTo( joint1, joint2 );
        double const std_dev = sqrt( distance.getCovariance()( 0, 0 ) );
        double const mean = distance + offset;

        return SoftClassification::sampleIntervals( &intervals[0], &intervals[0] + intervals.size(), mean, std_dev );
    }

} // psychophysical

class PsychophysicalFeatureRecognizer : public SpatialFeatureRecognizer
{
public:
    typedef humanoid::HumanoidFeatureRecognizer _HumanoidFeatureRecognizer;
    typedef humanoid::Humanoid _Humanoid;
    typedef humanoid::_SoftClassificationSet _SoftClassificationSet;

protected:
    XmlRpc::XmlRpcValue params_;

public:
    template<class... __Args>
    PsychophysicalFeatureRecognizer( _Humanoid const & humanoid, __Args&&... args )
    :
        SpatialFeatureRecognizer( humanoid )
    {
        init( args... );
    }

    template<class... __Args>
    void init( ros::NodeHandle & nh, __Args&&... args )
    {
        // { voice_loudness: [ { name: silent, min: 0, max: 0.01 }, { name: very_soft, min: 0.01, max: 0.1 } ] }
        params_ = quickdev::ParamReader::readParam<XmlRpc::XmlRpcValue>( nh, "/proxemics/psychophysical" );
        init( args... );
    }

    template<class... __Args>
    void init( XmlRpc::XmlRpcValue params, __Args&&... args )
    {
        params_ = params;
        init( args... );
    }

    void init(){}

    void appendClassification( _PsychophysicalDimensionMsg & dimension_msg, _SoftClassificationSet const & classification )
    {
        for( auto classification_it = classification.cbegin(); classification_it != classification.cend(); ++classification_it )
        {
            dimension_msg.classifications.push_back( *classification_it );
        }
    }

    _SoftClassificationSet getSexClassification()
    {
        return proxemics::psychophysical::getSexClassification();
    }

    _SoftClassificationSet getPosturalClassification()
    {
        return proxemics::psychophysical::getPosturalClassification();
    }

    _SoftClassificationSet getSFPClassification( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "neck", std::string const & to_joint_name = "" )
    {
        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

//        quickdev::start_stream_indented( "getting sfp classification from ", humanoid_.name, " to ", other.humanoid_.name );
        _SoftClassificationSet result = proxemics::psychophysical::getSFPClassification( from_joint, to_joint, params_["sfp"] );
//        quickdev::end_stream_indented();

        return result;
    }

    _SoftClassificationSet getKinestheticClassification( _HumanoidFeatureRecognizer const & other )
    {
        // average distance from neck to r/l shoulder
        auto const neck_joint = humanoid_["neck"];
        auto const pelvis_joint = humanoid_["pelvis"];
        auto const torso_joint = humanoid_["torso"];
        auto const left_shoulder_joint = humanoid_["left_shoulder"];
        auto const right_shoulder_joint = humanoid_["right_shoulder"];
        auto const left_elbow_joint = humanoid_["left_elbow"];
        auto const right_elbow_joint = humanoid_["right_elbow"];
        auto const left_hand_joint = humanoid_["left_hand"];
        auto const right_hand_joint = humanoid_["right_hand"];

        auto const body_max = ( proxemics::spatial::getDistanceTo( neck_joint, left_shoulder_joint ) + proxemics::spatial::getDistanceTo( neck_joint, right_shoulder_joint ) ) * 0.5;
        auto const body_outside_max = body_max + double( Meter( Inch( 1 ) ) );

        auto const lower_arm_max = body_max + ( proxemics::spatial::getDistanceTo( left_elbow_joint, left_hand_joint ) + proxemics::spatial::getDistanceTo( right_elbow_joint, right_hand_joint ) ) * 0.5;
        auto const lower_arm_outside_max = lower_arm_max + double( Meter( Inch( 2 ) ) );

        auto const whole_arm_max = lower_arm_max + ( proxemics::spatial::getDistanceTo( left_shoulder_joint, left_elbow_joint ) + proxemics::spatial::getDistanceTo( right_shoulder_joint, right_elbow_joint ) ) * 0.5;
        auto const whole_arm_outside_max = whole_arm_max + double( Meter( Inch( 1 ) ) );

        auto const reach_max = whole_arm_max + proxemics::spatial::getDistanceTo( torso_joint, pelvis_joint ) + proxemics::spatial::getDistanceTo( neck_joint, torso_joint );
        auto const reach_outside_max = reach_max + double( Meter( Inch( 4 ) ) );

        auto const offset = quickdev::ParamReader::getXmlRpcValue<double>( params_["kinesthetic"], "offset", 0 );

        auto const distance = proxemics::spatial::getDistance2DTo( humanoid_["neck"], other.humanoid_[getClosestJoint( "neck", other )] );
        double const base_std_dev = sqrt( distance.getCovariance()( 0, 0 ) );
        double const mean = distance + offset;

        _SoftClassificationSet result;

        double std_dev;
/*
        PRINT_INFO( "interval [body_max]: 0 -> %f\n", body_max.getFeature() );
        PRINT_INFO( "interval [body_outside_max]: %f -> %f", body_max.getFeature(), body_outside_max.getFeature() );
        PRINT_INFO( "interval [lower_arm_max]: %f -> %f", body_outside_max.getFeature(), lower_arm_max.getFeature() );
        PRINT_INFO( "interval [lower_arm_outside_max]: %f -> %f", lower_arm_max.getFeature(), lower_arm_outside_max.getFeature() );
        PRINT_INFO( "interval [whole_arm_max]: %f -> %f", lower_arm_outside_max.getFeature(), whole_arm_max.getFeature() );
        PRINT_INFO( "interval [whole_arm_outside_max]: %f -> %f", whole_arm_max.getFeature(), whole_arm_outside_max.getFeature() );
        PRINT_INFO( "interval [reach_max]: %f -> %f", whole_arm_outside_max.getFeature(), reach_max.getFeature() );
        PRINT_INFO( "interval [reach_outside_max]: %f -> %f", reach_max.getFeature(), reach_outside_max.getFeature() );
        PRINT_INFO( "interval [outside]: %f -> inf", reach_outside_max.getFeature() );
*/
        std_dev = base_std_dev + sqrt( body_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 0, cdf_unsigned_normal( mean, std_dev, body_max ) -                           cdf_unsigned_normal( mean, std_dev, 0.0 ),                   "body" ) );

        std_dev = base_std_dev + sqrt( body_outside_max.getCovariance()( 0, 0 ) + body_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 1, cdf_unsigned_normal( mean, std_dev, body_outside_max ) -                   cdf_unsigned_normal( mean, std_dev, body_max ),              "body_outside" ) );

        std_dev = base_std_dev + sqrt( lower_arm_max.getCovariance()( 0, 0 ) + body_outside_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 2, cdf_unsigned_normal( mean, std_dev, lower_arm_max ) -                      cdf_unsigned_normal( mean, std_dev, body_outside_max ),      "lower_arm" ) );

        std_dev = base_std_dev + sqrt( lower_arm_outside_max.getCovariance()( 0, 0 ) + lower_arm_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 3, cdf_unsigned_normal( mean, std_dev, lower_arm_outside_max ) -              cdf_unsigned_normal( mean, std_dev, lower_arm_max ),         "lower_arm_outside" ) );

        std_dev = base_std_dev + sqrt( whole_arm_max.getCovariance()( 0, 0 ) + lower_arm_outside_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 4, cdf_unsigned_normal( mean, std_dev, whole_arm_max ) -                      cdf_unsigned_normal( mean, std_dev, lower_arm_outside_max ), "whole_arm" ) );

        std_dev = base_std_dev + sqrt( whole_arm_outside_max.getCovariance()( 0, 0 ) + whole_arm_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 5, cdf_unsigned_normal( mean, std_dev, whole_arm_outside_max ) -              cdf_unsigned_normal( mean, std_dev, whole_arm_max ),         "whole_arm_outside" ) );

        std_dev = base_std_dev + sqrt( reach_max.getCovariance()( 0, 0 ) + whole_arm_outside_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 6, cdf_unsigned_normal( mean, std_dev, reach_max ) -                          cdf_unsigned_normal( mean, std_dev, whole_arm_outside_max ), "reach" ) );

        std_dev = base_std_dev + sqrt( reach_outside_max.getCovariance()( 0, 0 ) + reach_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 7, cdf_unsigned_normal( mean, std_dev, reach_outside_max ) -                  cdf_unsigned_normal( mean, std_dev, reach_max ),             "reach_outside" ) );

        std_dev = base_std_dev + sqrt( reach_outside_max.getCovariance()( 0, 0 ) );
        result.insert( SoftClassification( 8, 1 -                                                                        cdf_unsigned_normal( mean, std_dev, reach_outside_max ),     "outside" ) );

        return result;
    }

    _SoftClassificationSet getTouchClassification( _HumanoidFeatureRecognizer const & other )
    {
        auto const closest_pair = getClosestPair( other );

        auto const & from_joint = humanoid_[closest_pair.first];
        auto const & to_joint = other.humanoid_[closest_pair.second];

        _SoftClassificationSet result = proxemics::psychophysical::getTouchClassification( from_joint, to_joint, params_["touch"] );

        return result;
    }

    _SoftClassificationSet getVisualClassification( _HumanoidFeatureRecognizer const & other )
    {
        auto const & from_joint = humanoid_["eyes"];
        auto const & to_joint = other.humanoid_["eyes"];

//        quickdev::start_stream_indented( "getting visual classification from ", humanoid_.name, " to ", other.humanoid_.name );
        _SoftClassificationSet result = proxemics::psychophysical::getVisualClassification( from_joint, to_joint, params_["visual"] );
//        quickdev::end_stream_indented();

        return result;
    }

    _SoftClassificationSet getThermalClassification( _HumanoidFeatureRecognizer const & other )
    {
        auto const closest_pair = getClosestPair( other );

        auto const & from_joint = humanoid_[closest_pair.first];
        auto const & to_joint = other.humanoid_[closest_pair.second];

//        PRINT_INFO( "determining thermal classification from %s/%s -> %s/%s", humanoid_.name.c_str(), closest_pair.first.c_str(), other.humanoid_.name.c_str(), closest_pair.second.c_str() );

        _SoftClassificationSet result = proxemics::psychophysical::getThermalClassification( from_joint, to_joint, params_["thermal"] );

        return result;
    }

    _SoftClassificationSet getOlfactionClassification( _HumanoidFeatureRecognizer const & other )
    {
        auto const & from_joint = humanoid_["nose"];

        // get the closest joint in other to our nose
        auto const closest_joint = _HumanoidFeatureRecognizer::getClosestJoint( "nose", other );

//        PRINT_INFO( "determining olfaction classification from %s/%s -> %s/%s", humanoid_.name.c_str(), "nose", other.humanoid_.name.c_str(), closest_joint.c_str() );

        auto const & to_joint = other.humanoid_[closest_joint];

        _SoftClassificationSet result = proxemics::psychophysical::getOlfactionClassification( from_joint, to_joint, params_["olfaction"] );

        return result;
    }

    _SoftClassificationSet getVoiceLoudnessClassification( _HumanoidFeatureRecognizer const & other )
    {
        auto const & from_joint = humanoid_["head"];
        auto const & to_joint = other.humanoid_["mouth"];

        _SoftClassificationSet result = proxemics::psychophysical::getVoiceLoudnessClassification( from_joint, to_joint, params_["voice_loudness"] );

        return result;
    }

    _PsychophysicalFeatureMsg createMessage( _HumanoidFeatureRecognizer const & other, std::string const & from_joint_name = "pelvis", std::string const & to_joint_name = "" )
    {
        _PsychophysicalFeatureMsg psychophysical_feature_msg;
        psychophysical_feature_msg.header.stamp = ros::Time::now();
        psychophysical_feature_msg.observer_name = humanoid_.name;
        psychophysical_feature_msg.target_name = other.humanoid_.name;
        psychophysical_feature_msg.dimensions.resize( 9 );

        auto const & from_joint = humanoid_[from_joint_name];
        auto const & to_joint = other.humanoid_[to_joint_name.empty() ? from_joint_name : to_joint_name];

        /* ordering of dimensions according to Hall
        1) postural - sex identifiers
        2) sociofugal - sociopetal orientation (SFP axis)
        3) kinesthetic factors
        4) touch code
        5) retinal combinations
        6) thermal code
        7) olfaction code
        8) voice loudness scale
        */

        // -------------------------------------------------------------------------------------------------------------------------------------
        // sex
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[0];
            dimension_msg.name = "sex";

            auto const & classification = getSexClassification();
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // postural
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[1];
            dimension_msg.name = "postural";

            auto const & classification = getPosturalClassification();
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // sfp
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[2];
            dimension_msg.name = "sfp";

            auto const & classification = getSFPClassification( other, from_joint_name, to_joint_name );
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // kinesthetic
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[3];
            dimension_msg.name = "kinesthetic";

            auto const & classification = getKinestheticClassification( other );
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // touch
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[4];
            dimension_msg.name = "touch";

            auto const & classification = getTouchClassification( other );
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // visual
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[5];
            dimension_msg.name = "visual";

            auto const & classification = getVisualClassification( other );
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // thermal
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[6];
            dimension_msg.name = "thermal";

            auto const & classification = getThermalClassification( other );
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // olfaction
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[7];
            dimension_msg.name = "olfaction";

            auto const & classification = getOlfactionClassification( other );
            appendClassification( dimension_msg, classification );
        }

        // -------------------------------------------------------------------------------------------------------------------------------------
        // voice loudness
        {
            auto & dimension_msg = psychophysical_feature_msg.dimensions[8];
            dimension_msg.name = "voice_loudness";

            auto const & classification = getVoiceLoudnessClassification( other );
            appendClassification( dimension_msg, classification );
        }

        return psychophysical_feature_msg;
    }
};

} // proxemics

#endif // PROXEMICMODELS_PSYCHOPHYSICALFEATURES_H_
