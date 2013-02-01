/***************************************************************************
 *  include/humanoid/spatial_metrics.h
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

#ifndef HUMANOIDMODELS_HUMANOID_SPATIALMETRICS_H_
#define HUMANOIDMODELS_HUMANOID_SPATIALMETRICS_H_

#include <quickdev/geometry_message_conversions.h>
#include <quickdev/numeric_unit_conversions.h>

#include <humanoid/humanoid_features.h>

#include <tf/LinearMath/Vector3.h>

namespace humanoid
{
namespace spatial
{
    using humanoid::_Humanoid;
    using humanoid::_HumanoidPair;
    using humanoid::_HumanoidJointMsg;

    static double getDistanceBetween( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        tf::Vector3 joint1_pos = unit::make_unit( joint1 );
        tf::Vector3 joint2_pos = unit::make_unit( joint2 );

        return joint1_pos.distance( joint2_pos );
    }

    static tf::Vector3 getDistance( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        tf::Vector3 joint1_pos = unit::make_unit( joint1 );
        tf::Vector3 joint2_pos = unit::make_unit( joint2 );

        return joint2_pos - joint1_pos;
    }

    static tf::Vector3 getMidpoint( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        tf::Vector3 joint1_pos = unit::make_unit( joint1 );
        tf::Vector3 joint2_pos = unit::make_unit( joint2 );

        return 0.5 * ( joint1_pos + joint2_pos );
    }

    static tf::Transform getTransformTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        tf::Transform joint1_tf = unit::make_unit( joint1 );
        tf::Transform joint2_tf = unit::make_unit( joint2 );

        return joint1_tf.inverse() * joint2_tf;
    }

    static tf::Quaternion getAngleTo( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        return getTransformTo( joint1, joint2 ).getRotation();
    }

    static tf::Vector3 getAngleToRPY( _HumanoidJointMsg const & joint1, _HumanoidJointMsg const & joint2 )
    {
        return unit::convert<tf::Vector3>( getAngleTo( joint1, joint2 ) );
    }
} // spatial
} // humanoid

#endif // HUMANOIDMODELS_HUMANOID_SPATIALMETRICS_H_
