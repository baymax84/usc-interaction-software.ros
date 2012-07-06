/***************************************************************************
 *  include/proxemic_models/zonal_features.h
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

#ifndef PROXEMICMODELS_ZONALFEATURES_H_
#define PROXEMICMODELS_ZONALFEATURES_H_

#include <proxemic_models/spatial_features.h>

#include <proxemic_models/ZonalFeatureArray.h>

namespace proxemics
{
    typedef proxemic_models::ZonalFeatureArray _ZonalFeatureArrayMsg;
    typedef proxemic_models::ZonalFeature _ZonalFeatureMsg;

namespace zonal
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;
    using humanoid::_SoftClassificationSet;

    // =========================================================================================================================================
    static _SoftClassificationSet getDistanceClassification( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2, XmlRpc::XmlRpcValue & intervals )
    {
        _SoftClassificationSet result;

        auto const distance = proxemics::spatial::getDistance2DTo( joint1, joint2 );
        double const sigma = sqrt( distance.getCovariance()( 0, 0 ) );
        double const mean = distance;

        return SoftClassification::sampleIntervals( &intervals[0], &intervals[0] + intervals.size(), mean, sigma );
    }
} // zonal

class ZonalFeatureRecognizer : public humanoid::HumanoidFeatureRecognizer
{
public:
    typedef humanoid::HumanoidFeatureRecognizer _HumanoidFeatureRecognizer;
    typedef humanoid::Humanoid _Humanoid;
    typedef humanoid::_SoftClassificationSet _SoftClassificationSet;

protected:
    XmlRpc::XmlRpcValue params_;

public:
    ZonalFeatureRecognizer(){}

    template<class... __Args>
    ZonalFeatureRecognizer( _Humanoid const & humanoid, __Args&&... args )
    :
        _HumanoidFeatureRecognizer( humanoid )
    {
        init( args... );
    }

    template<class... __Args>
    void init( ros::NodeHandle & nh, __Args&&... args )
    {
        // { voice_loudness: [ { name: silent, min: 0, max: 0.01 }, { name: very_soft, min: 0.01, max: 0.1 } ] }
        params_ = quickdev::ParamReader::readParam<XmlRpc::XmlRpcValue>( nh, "/proxemics/zonal" );
        init( args... );
    }

    template<class... __Args>
    void init( XmlRpc::XmlRpcValue params, __Args&&... args )
    {
        params_ = params;
        init( args... );
    }

    void init(){}

    _SoftClassificationSet getDistanceClassification( _HumanoidFeatureRecognizer const & other )
    {
        auto const & from_joint = humanoid_["torso"];
        auto const & to_joint = other.humanoid_["torso"];

        _SoftClassificationSet result = proxemics::zonal::getDistanceClassification( from_joint, to_joint, params_["intervals"]["distance"] );

        return result;
    }

    _ZonalFeatureMsg createMessage( _HumanoidFeatureRecognizer const & other )
    {
        _ZonalFeatureMsg zonal_feature_msg;
        zonal_feature_msg.header.stamp = ros::Time::now();
        zonal_feature_msg.observer_name = humanoid_.name;
        zonal_feature_msg.target_name = other.humanoid_.name;

        auto const classifications = getDistanceClassification( other );

        for( auto classification_it = classifications.cbegin(); classification_it != classifications.cend(); ++classification_it )
        {
            zonal_feature_msg.classifications.push_back( *classification_it );
        }

        return zonal_feature_msg;
    }

};

} // proxemics

#endif // PROXEMICMODELS_ZONALFEATURES_H_
