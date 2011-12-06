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

#include <humanoid/humanoid_features.h>
#include <LinearMath/btVector3.h>

using humanoid::_Humanoid;
using humanoid::_HumanoidPair;
using humanoid::_HumanoidJointMsg;

namespace proxemics
{
    class SpatialFeature
    {
    public:
        struct FeatureVec
        {
            _HumanoidJointMsg joint1;
            _HumanoidJointMsg joint2;
            btVector3 point1;
            btVector3 point2;
            double distance;
            btVector3 midpoint;
            double orientation1;
            double orientation2;
        };

        typedef FeatureVec _FeatureVec;

    protected:
        _FeatureVec feature_vec_;

    QUICKDEV_DECLARE_ACCESSOR2( feature_vec_, FeatureVec )

    public:
        template<class... __Args>
        SpatialFeature( __Args&&... args )
        :
            feature_vec_( calculateFeatureVec( args... ) )
        {
            //
        }

    private:
        static _FeatureVec calculateFeatureVec( const _HumanoidPair & pair )
        {
            const auto & joint1 = pair.first["torso"];
            const auto & joint2 = pair.second["torso"];

            const btVector3 joint1_vec(
                    joint1.pose.pose.position.x,
                    joint1.pose.pose.position.y,
                    0 );

            const btVector3 joint2_vec(
                joint2.pose.pose.position.x,
                joint2.pose.pose.position.y,
                0 );

            return _FeatureVec { joint1, joint2, joint1_vec, joint2_vec, joint1_vec.distance( joint2_vec ), 0.5 * ( joint1_vec + joint2_vec ), 0, 0 };
        }
    };
}

#endif // PROXEMICMODELS_SPATIALFEATURES_H_
