/***************************************************************************
 *  include/proxemic_recognizers/hall_recognizer.h
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

#ifndef PROXEMICRECOGNIZERS_HALLRECOGNIZER_H_
#define PROXEMICRECOGNIZERS_HALLRECOGNIZER_H_

#include <quickdev/node.h>

#include <proxemic_models/psychophysical_features.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

using humanoid::_HumanoidStateMsg;
using proxemics::_PsychophysicalFeatureArrayMsg;

typedef HumanoidRecognizerPolicy<_PsychophysicalFeatureArrayMsg> _HumanoidRecognizerPolicy;
QUICKDEV_DECLARE_NODE( HallRecognizer, _HumanoidRecognizerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( HallRecognizer )
{
    _MarkerMsg marker_template_;
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( HallRecognizer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initAll();

        marker_template_.header.frame_id = "/openni_depth_tracking_frame";
        marker_template_.ns = "mehrabian_visualization";
        marker_template_.action = visualization_msgs::Marker::ADD;
        marker_template_.lifetime = ros::Duration( 0.1 );
        marker_template_.pose.orientation.w = 1.0;
    }

    QUICKDEV_SPIN_ONCE()
    {
        QUICKDEV_LOCK_CACHE_AND_GET( states_cache_, states_msg );
        if( !states_msg ) return;

        auto const now = ros::Time::now();

        _MarkerArrayMsg markers_msg;

        unsigned int current_id = 0;

        // we can't serialize maps (thanks, ROS) so we have to rebuild this every iteration
        _HumanoidRecognizerPolicy::updateHumanoids( states_msg );
        _HumanoidRecognizerPolicy::updateHumanoidPairs();

        auto const & humanoid_pairs = _HumanoidRecognizerPolicy::getHumanoidPairs();

        for( auto pair = humanoid_pairs.cbegin(); pair != humanoid_pairs.cend(); ++pair )
        {
            //const auto & joint1 = _HumanoidRecognizerPolicy::states_map_[pair->first.name]["torso"];
            //const auto & joint2 = _HumanoidRecognizerPolicy::states_map_[pair->second.name]["torso"];

            /*std_msgs::ColorRGBA current_color;
            current_color.r = 0.0;
            current_color.g = 0.0;
            current_color.b = 1.0;
            current_color.a = 1.0;

            _MarkerMsg arrow_marker = arrow_marker_template_;
            arrow_marker.header.stamp = now;
            arrow_marker.id = current_id ++;
            arrow_marker.color = current_color;
            arrow_marker.points.push_back( joint1.pose.pose.position );
            arrow_marker.points.push_back( joint2.pose.pose.position );

            markers_msg.markers.push_back( arrow_marker );*/
        }
    }
};

#endif // PROXEMICRECOGNIZERS_HALLRECOGNIZER_H_
