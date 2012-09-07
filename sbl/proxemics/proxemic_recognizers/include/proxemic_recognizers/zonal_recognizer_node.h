/***************************************************************************
 *  include/proxemic_recognizers/zonal_recognizer_node.h
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

#ifndef PROXEMICRECOGNIZERS_ZONALRECOGNIZERNODE_H_
#define PROXEMICRECOGNIZERS_ZONALRECOGNIZERNODE_H_

#include <quickdev/node.h>

#include <proxemic_models/zonal_features.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

using humanoid::_HumanoidStateMsg;
using proxemics::_ZonalFeatureMsg;
using proxemics::_ZonalFeatureArrayMsg;

typedef HumanoidRecognizerPolicy<_ZonalFeatureArrayMsg> _HumanoidRecognizerPolicy;

QUICKDEV_DECLARE_NODE( ZonalRecognizer, _HumanoidRecognizerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ZonalRecognizer )
{
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ZonalRecognizer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<_HumanoidRecognizerPolicy>( "update_pairs_param", true );
        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const now = ros::Time::now();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        _ZonalFeatureArrayMsg features_msg;

        auto const & humanoid_pairs = _HumanoidRecognizerPolicy::getHumanoidPairs();

        for( auto pair = humanoid_pairs.cbegin(); pair != humanoid_pairs.cend(); ++pair )
        {
            auto feature1( proxemics::ZonalFeatureRecognizer( pair->first, nh_rel ) );
            auto feature2( proxemics::ZonalFeatureRecognizer( pair->second, nh_rel ) );

            features_msg.features.push_back( feature1.createMessage( feature2 ) );
            features_msg.features.push_back( feature2.createMessage( feature1 ) );
        }

        _HumanoidRecognizerPolicy::updateFeatures( features_msg );
    }
};

#endif // PROXEMICRECOGNIZERS_ZONALRECOGNIZERNODE_H_
