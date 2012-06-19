/***************************************************************************
 *  include/proxemic_recognizers/mehrabian_recognizer.h
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

#ifndef PROXEMICRECOGNIZERS_MEHRABIANRECOGNIZER_H_
#define PROXEMICRECOGNIZERS_MEHRABIANRECOGNIZER_H_

#include <quickdev/node.h>

#include <proxemic_models/spatial_features.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

using humanoid::_HumanoidStateMsg;
using proxemics::_SpatialFeatureArrayMsg;

typedef HumanoidRecognizerPolicy<_SpatialFeatureArrayMsg> _HumanoidRecognizerPolicy;
QUICKDEV_DECLARE_NODE( MehrabianRecognizer, _HumanoidRecognizerPolicy )

typedef _HumanoidRecognizerPolicy::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;
typedef _HumanoidRecognizerPolicy::_MarkerArrayMsg _MarkerArrayMsg;

QUICKDEV_DECLARE_NODE_CLASS( MehrabianRecognizer )
{
private:
    _MarkerMsg
        marker_template_,
        lines_marker_template_,
        arrow_marker_template_,
        text_marker_template_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( MehrabianRecognizer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<_HumanoidRecognizerPolicy>( "update_pairs_param", true );

        marker_template_.header.frame_id = "/openni_depth_tracking_frame";
        marker_template_.ns = "mehrabian_visualization";
        marker_template_.action = visualization_msgs::Marker::ADD;
        marker_template_.lifetime = ros::Duration( 0.1 );
        marker_template_.pose.orientation.w = 1.0;

        lines_marker_template_ = marker_template_;
        lines_marker_template_.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker_template_.scale.x = 0.02;

        arrow_marker_template_ = marker_template_;
        arrow_marker_template_.type = visualization_msgs::Marker::ARROW;
        arrow_marker_template_.scale.x = 0.01;
        arrow_marker_template_.scale.y = 0.02;

        text_marker_template_ = marker_template_;
        text_marker_template_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker_template_.scale.z = 0.1;

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const now = ros::Time::now();

        _MarkerArrayMsg markers_msg;
        _SpatialFeatureArrayMsg features_msg;

        unsigned int marker_id = 0;

        auto const humanoid_pairs = _HumanoidRecognizerPolicy::getHumanoidPairs();

        for( auto pair = humanoid_pairs.cbegin(); pair != humanoid_pairs.cend(); ++pair )
        {
            const auto & spatial_feature = proxemics::SpatialFeature( pair->first["torso"], pair->second["torso"] );
            features_msg.features.push_back( spatial_feature );
            //const auto & joint1 = pair->first["torso"];
            //const auto & joint2 = pair->second["torso"];

            std_msgs::ColorRGBA current_color;
            current_color.r = 0.0;
            current_color.g = 1.0;
            current_color.b = 1.0;
            current_color.a = 1.0;

            _MarkerMsg lines_marker = lines_marker_template_;
            lines_marker.header.stamp = now;
            lines_marker.id = marker_id ++;
            lines_marker.color = current_color;
            lines_marker.points.push_back( spatial_feature.joint1.pose.pose.position );
            lines_marker.points.push_back( spatial_feature.joint2.pose.pose.position );

            markers_msg.markers.push_back( lines_marker );

            _MarkerMsg text_marker = text_marker_template_;
            text_marker.header.stamp = now;
            text_marker.id = marker_id ++;
            text_marker.color = current_color;

            char buffer[10];
            sprintf( buffer, "%.2f", spatial_feature.distance );
            text_marker.text = buffer;
            text_marker.pose.position.x = spatial_feature.midpoint.getX();
            text_marker.pose.position.y = spatial_feature.midpoint.getY();
            text_marker.pose.position.z = spatial_feature.midpoint.getZ() - 0.25;

            markers_msg.markers.push_back( text_marker );
        }

        _HumanoidRecognizerPolicy::updateMarkers( markers_msg );
        _HumanoidRecognizerPolicy::updateFeatures( features_msg );
    }
};

#endif // PROXEMICRECOGNIZERS_MEHRABIANRECOGNIZER_H_
