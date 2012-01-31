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

#include <quickdev/convolved_struct.h>
#include <quickdev/geometry_message_conversions.h>

#include <humanoid/humanoid_features.h>

#include <LinearMath/btVector3.h>

#include <proxemic_models/SpatialFeatureArray.h>

namespace proxemics
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;

    DECLARE_CONVOLVED_STRUCT_BASE( SpatialFeature )
    {
        //! The target joint from the first humanoid
        _HumanoidJointMsg joint1;
        //! The target joint from the second humanoid
        _HumanoidJointMsg joint2;

        //! The location of the first joint
        /*! We use a btVector3 to enable easy geometric calculations */
        btVector3 point1;
        //! The location of the second joint
        /*! We use a btVector3 to enable easy geometric calculations */
        btVector3 point2;
        //! The distance between the two joints
        double distance;
        //! The midpoint between the two joints
        btVector3 midpoint;
        //! The orientation of the first joint with respect to the second
        btQuaternion orientation1;
        //! The orientation of the second joint with respect to the first
        btQuaternion orientation2;

        DECLARE_CONVOLVED_STRUCT_TYPES( SpatialFeature, _HumanoidJointMsg, _HumanoidJointMsg );

        INST_CONVOLVED_STRUCT( SpatialFeature ),
            INST_CONVOLVED_STRUCT_VAR( 0, joint1 ),
            INST_CONVOLVED_STRUCT_VAR( 1, joint2 )
        {
            point1 = btVector3( joint1.pose.pose.position.x, joint1.pose.pose.position.y, 0 );
            point2 = btVector3( joint2.pose.pose.position.x, joint2.pose.pose.position.y, 0 );

            updateAll();
        }

        //! Re-calculate all feature components
        void updateAll()
        {
            distance = calculateDistance();
            midpoint = calculateMidpoint();
            orientation1 = calculateOrientation1();
            orientation2 = calculateOrientation2();
        }

        //! Create a SpatialFeature from a pair of humanoids
        /*! Default behavior is to build feature data from the torso joints of the humanoids. This behavior can be overridden using
         *  SpatialFeature() */
        static SpatialFeature fromHumanoidPair( const _HumanoidPair & pair )
        {
            return SpatialFeature( pair.first["torso"], pair.second["torso"] );
        }

        //! Get the distance between two joints
        auto calculateDistance() -> decltype( distance )
        {
            return point1.distance( point2 );
        }

        //! Get the midpoint between two joints
        auto calculateMidpoint() -> decltype( midpoint )
        {
            return 0.5 * ( point1 + point2 );
        }

        //! Get the orientation of joint1 with respect to joint2
        /*! \note unimplemented */
        auto calculateOrientation1() -> decltype( orientation1 )
        {
            return btQuaternion();
        }

        //! Get the orientation of joint2 with respect to joint1
        /*! \note unimplemented */
        auto calculateOrientation2() -> decltype( orientation2 )
        {
            return btQuaternion();
        }
    };

    typedef SpatialFeature _SpatialFeature;
    typedef proxemic_models::SpatialFeature _SpatialFeatureMsg;
    typedef proxemic_models::SpatialFeatureArray _SpatialFeatureArrayMsg;
} // proxemics

// convert from a spatial feature struct to a spatial feature message
DECLARE_UNIT_CONVERSION_LAMBDA( proxemics::_SpatialFeature, proxemics::_SpatialFeatureMsg, vec, proxemics::_SpatialFeatureMsg msg; msg.joint1 = vec.joint1; msg.joint2 = vec.joint2; msg.distance = vec.distance; msg.midpoint = unit::make_unit( vec.midpoint ); msg.orientation1 = unit::make_unit( vec.orientation1 ); msg.orientation2 = unit::make_unit( vec.orientation2 ); return msg; )
//DECLARE_UNIT_CONVERSION_LAMBDA( _SpatialFeatureMsg, _SpatialFeature, msg, _SpatialFeature vec; vec.distance = msg.distance; vec.midpoint = msg.midpoint; vec.orientation1 = msg.orientation1; vec.orientation2 = msg.orientation2; return vec; )

#endif // PROXEMICMODELS_SPATIALFEATURES_H_
