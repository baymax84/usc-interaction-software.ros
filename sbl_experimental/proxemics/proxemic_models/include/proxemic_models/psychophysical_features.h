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

#include <proxemic_models/PsychophysicalFeatureArray.h>

namespace proxemics
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;

    DECLARE_CONVOLVED_STRUCT( PsychophysicalFeature, SpatialFeature )
    {
        int sex_code;
        int posture_code;
        int sfp_axis_code;
        int kinesthetic_code;
        int touch_code;
        int visual_code;
        int thermal_code;
        int olfaction_code;
        int voice_loudness_code;
        int distance_code;

        DECLARE_CONVOLVED_STRUCT_TYPES( PsychophysicalFeature, int );

        INST_CONVOLVED_STRUCT( PsychophysicalFeature ),
            INST_CONVOLVED_STRUCT_VAR( 0, sex_code )
        {
            updateAll();
        }

        void updateAll()
        {
            posture_code = calculatePostureCode();
            sfp_axis_code = calculateSfpAxisCode();
            kinesthetic_code = calculateKinestheticCode();
            touch_code = calculateTouchCode();
            visual_code = calculateVisualCode();
            thermal_code = calculateThermalCode();
            olfaction_code = calculateOlfactionCode();
            voice_loudness_code = calculateVoiceLoudnessCode();
            distance_code = calculateDistanceCode();
        }

        static PsychophysicalFeature fromHumanoidPair( const _HumanoidPair & pair )
        {
            return PsychophysicalFeature( SpatialFeature::fromHumanoidPair( pair ), 0 );
        }

        auto calculatePostureCode() -> decltype( posture_code )
        {
            return decltype( posture_code )();
        }

        auto calculateSfpAxisCode() -> decltype( sfp_axis_code )
        {
            return decltype( sfp_axis_code )();
        }

        auto calculateKinestheticCode() -> decltype( kinesthetic_code )
        {
            return decltype( kinesthetic_code )();
        }

        auto calculateTouchCode() -> decltype( touch_code )
        {
            return decltype( touch_code )();
        }

        auto calculateVisualCode() -> decltype( visual_code )
        {
            return decltype( visual_code )();
        }

        auto calculateThermalCode() -> decltype( thermal_code )
        {
            return decltype( thermal_code )();
        }

        auto calculateOlfactionCode() -> decltype( olfaction_code )
        {
            return decltype( olfaction_code )();
        }

        auto calculateVoiceLoudnessCode() -> decltype( voice_loudness_code )
        {
            return decltype( voice_loudness_code )();
        }

        auto calculateDistanceCode() -> decltype( distance_code )
        {
            return decltype( distance_code )();
        }
    };

    typedef PsychophysicalFeature _PsychophysicalFeature;
    typedef proxemic_models::PsychophysicalFeature _PsychophysicalFeatureMsg;
    typedef proxemic_models::PsychophysicalFeatureArray _PsychophysicalFeatureArrayMsg;
}

//DECLARE_UNIT_CONVERSION_LAMBDA( proxemics::_PsychophysicalFeature, proxemics::_PsychophysicalFeatureMsg, vec, proxemics::_PsychophysicalFeatureMsg msg; msg.joint1 = vec.joint1; msg.joint2 = vec.joint2; msg.distance = vec.distance; msg.midpoint = unit::make_unit( vec.midpoint ); msg.orientation1 = unit::make_unit( vec.orientation1 ); msg.orientation2 = unit::make_unit( vec.orientation2 ); return msg; )

#endif // PROXEMICMODELS_PSYCHOPHYSICALFEATURES_H_
