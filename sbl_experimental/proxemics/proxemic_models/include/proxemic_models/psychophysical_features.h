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

namespace proxemics
{
namespace psychophysical
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;

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

    static SexCode getSexCode( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        return SEX_MALE;
    }

    static PostureCode getPostureCode( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        return POSTURE_STANDING;
    }

    static SfpAxisCode getSfpAxisCode( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        Radian const & yaw_rad = fabs( angles::normalize_angle( proxemics::spatial::getAngleToRPY( joint1, joint2 ).getZ() ) );
        Degree const & yaw_deg( yaw_rad );

        int const num_sfp_axes = 8;
        double const interval_angle = 180.0 / double( num_sfp_axes );

        double current_angle = 0.5 * interval_angle;
        int sfp_axis = -1;

        while( current_angle < 180.0 && yaw_deg > current_angle )
        {
            ++sfp_axis;
            current_angle += interval_angle;
        }

        return SfpAxisCode( ++sfp_axis );
    }
} // psychophysical
} // proxemics

#endif // PROXEMICMODELS_PSYCHOPHYSICALFEATURES_H_
