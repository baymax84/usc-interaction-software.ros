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
        btVector3 vec( 0, 0, 3.14 );
        btQuaternion quat( unit::make_unit( vec ) );
        btVector3 vec_res = unit::make_unit( quat );
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<_HumanoidRecognizerPolicy>( "update_pairs_param", true );

        marker_template_.header.frame_id = "/world";
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

        // copy
        auto const humanoid_pairs = _HumanoidRecognizerPolicy::getHumanoidPairs();

        for( auto pair = humanoid_pairs.cbegin(); pair != humanoid_pairs.cend(); ++pair )
        {
            PRINT_INFO( "analyzing pair [%s,%s]", pair->first.name.c_str(), pair->second.name.c_str() );
            auto const & feature1 = proxemics::SpatialFeatureRecognizer( pair->first );
            auto const & feature2 = proxemics::SpatialFeatureRecognizer( pair->second );

            auto const feature1_msg = feature1.createMessage( feature2 );
            auto const feature2_msg = feature2.createMessage( feature1 );

            features_msg.features.push_back( feature1_msg );
            features_msg.features.push_back( feature2_msg );
            //const auto & joint1 = pair->first["torso"];
            //const auto & joint2 = pair->second["torso"];

            std_msgs::ColorRGBA current_color;
            current_color.r = 0.0;
            current_color.g = 1.0;
            current_color.b = 1.0;
            current_color.a = 1.0;

            // implicit conversion from quickdev::FeatureWithCovariance<__Feature, __Dim__> to __Feature (with __Feaure = btVector3)
            btVector3 const feature1_origin = feature1.getOrigin();
            btVector3 const feature2_origin = feature2.getOrigin();

            _MarkerMsg lines_marker = lines_marker_template_;
            lines_marker.header.stamp = now;
            lines_marker.id = marker_id ++;
            lines_marker.color = current_color;
            lines_marker.points.push_back( unit::make_unit( feature1_origin ) );
            lines_marker.points.push_back( unit::make_unit( feature2_origin ) );

            btVector3 feature1_origin2d = btVector3( feature1_origin.getX(), feature1_origin.getY(), 0 );
            btVector3 feature2_origin2d = btVector3( feature2_origin.getX(), feature2_origin.getY(), 0 );

            btTransform const world_to_pelvis1_tf = unit::make_unit( pair->first["pelvis"] );
            btTransform const world_to_pelvis2_tf = unit::make_unit( pair->second["pelvis"] );

            btVector3 pelvis1_ori = unit::make_unit( world_to_pelvis1_tf.getRotation() );
            btVector3 pelvis2_ori = unit::make_unit( world_to_pelvis2_tf.getRotation() );

            //pelvis1_ori.setZ( 0 );
            //pelvis1_ori.setY( 0 );
            //pelvis2_ori.setZ( 0 );
            //pelvis2_ori.setY( 0 );

            btTransform world_to_feature1_x_tf = world_to_pelvis1_tf * btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( feature1_msg.pose.x, 0, 0 ) );
            btTransform world_to_feature2_x_tf = world_to_pelvis2_tf * btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( feature2_msg.pose.x, 0, 0 ) );

            // feature1_origin -> world_to_feature1_x_tf, world_to_feature1_x_tf -> feature2_origin2d (with z = feature1_origin.z)
            // feature2_origin -> world_to_feature2_x_tf, world_to_feature2_x_tf -> feature1_origin2d (with z = feature2_origin.z)

            lines_marker.points.push_back( unit::make_unit( feature1_origin ) );
            lines_marker.points.push_back( unit::make_unit( world_to_feature1_x_tf.getOrigin() ) );

            lines_marker.points.push_back( unit::make_unit( world_to_feature1_x_tf.getOrigin() ) );
            lines_marker.points.push_back( unit::make_unit( btVector3( feature2_origin.getX(), feature2_origin.getY(), feature1_origin.getZ() ) ) );

            lines_marker.points.push_back( unit::make_unit( feature2_origin ) );
            lines_marker.points.push_back( unit::make_unit( world_to_feature2_x_tf.getOrigin() ) );

            lines_marker.points.push_back( unit::make_unit( world_to_feature2_x_tf.getOrigin() ) );
            lines_marker.points.push_back( unit::make_unit( btVector3( feature1_origin.getX(), feature1_origin.getY(), feature2_origin.getZ() ) ) );

            markers_msg.markers.push_back( lines_marker );

            _MarkerMsg text_marker = text_marker_template_;
            text_marker.header.stamp = now;
            text_marker.id = marker_id ++;
            text_marker.color = current_color;

            btVector3 const features_midpoint = ( feature1_origin + feature2_origin ) / 2.0;

            char buffer[10];
            sprintf( buffer, "%.2f", feature1_origin.distance( feature2_origin ) );
            text_marker.text = buffer;
            text_marker.pose.position.x = features_midpoint.getX();
            text_marker.pose.position.y = features_midpoint.getY();
            text_marker.pose.position.z = features_midpoint.getZ() - 0.25;

            markers_msg.markers.push_back( text_marker );
        }

        _HumanoidRecognizerPolicy::updateMarkers( markers_msg );
        _HumanoidRecognizerPolicy::updateFeatures( features_msg );
    }
};

#endif // PROXEMICRECOGNIZERS_MEHRABIANRECOGNIZER_H_
