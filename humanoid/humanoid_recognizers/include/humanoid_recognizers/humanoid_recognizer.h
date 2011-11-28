/***************************************************************************
 *  include/humanoid_recognizers/humanoid_recognizer.h
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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_

#include <quickdev/node.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

#include <deque>

typedef HumanoidRecognizerPolicy _HumanoidRecognizerPolicy;
QUICKDEV_DECLARE_NODE( HumanoidRecognizer, _HumanoidRecognizerPolicy )

using humanoid::_PoseWithConfidenceMsg;
typedef _HumanoidRecognizerPolicy::_HumanoidStateArrayMsg _HumanoidStateArrayMsg;
typedef _HumanoidRecognizerPolicy::_MarkerArrayMsg _MarkerArrayMsg;
typedef _HumanoidRecognizerPolicy::_MarkerMsg _MarkerMsg;

QUICKDEV_DECLARE_NODE_CLASS( HumanoidRecognizer )
{
private:
    quickdev::MutexedCache<std::deque<_HumanoidStateArrayMsg::ConstPtr> > state_arrays_cache_;

    _MarkerMsg
        marker_template_,
        text_marker_template_,
        lines_marker_template_,
        points_marker_template_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( HumanoidRecognizer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initAll();
        _HumanoidRecognizerPolicy::registerCallback( quickdev::auto_bind( &HumanoidRecognizerNode::humanoidStatesCB, this ) );

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        auto & multi_pub = _HumanoidRecognizerPolicy::getMultiPub();
        multi_pub.addPublishers<_HumanoidStateArrayMsg>( nh_rel, { "humanoid_states_agg" } );

        marker_template_.header.frame_id = "/openni_depth_tracking_frame";
        marker_template_.ns = "basic_skeleton";
        marker_template_.action = visualization_msgs::Marker::ADD;
        marker_template_.lifetime = ros::Duration( 0.001 );
        marker_template_.pose.orientation.w = 1.0;

        text_marker_template_ = marker_template_;
        text_marker_template_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker_template_.scale.z = 0.1;

        lines_marker_template_ = marker_template_;
        lines_marker_template_.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker_template_.scale.x = 0.02;

        points_marker_template_ = marker_template_;
        points_marker_template_.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker_template_.scale.x = points_marker_template_.scale.y
                                        = points_marker_template_.scale.z
                                        = 0.05;
    }

    void appendLineMarker( _MarkerMsg & msg, const std::string & from, const std::string & to, const std::map<std::string, _PoseWithConfidenceMsg> & point_map )
    {
        const auto & from_point = point_map.find( from );
        const auto & to_point = point_map.find( to );

        if( from_point == point_map.end() || to_point == point_map.end() ) return;

        msg.points.push_back( from_point->second.pose.position );
        msg.points.push_back( to_point->second.pose.position );
    }

    QUICKDEV_SPIN_ONCE()
    {
        // get a read/write reference to the cache's data (called state_arrays_cache)
        QUICKDEV_LOCK_CACHE_AND_GET( state_arrays_cache_, state_arrays_cache );

        // if there are no items in the cache then we have no work to do
        if( state_arrays_cache.size() == 0 ) return;

        _HumanoidStateArrayMsg combined_states_msg;
        _MarkerArrayMsg markers;
        const auto now = ros::Time::now();

        unsigned int current_id = 0;

        for( auto state_array = state_arrays_cache.begin(); state_array != state_arrays_cache.end(); ++state_array )
        {
            // if the current state array is null, discard it and move on
            if( !( *state_array ) ) continue;

            for( auto state = (*state_array)->states.begin(); state != (*state_array)->states.end(); ++state )
            {
                combined_states_msg.states.push_back( *state );

                std::map<std::string, _PoseWithConfidenceMsg> point_map;

                std_msgs::ColorRGBA current_color;
                current_color.r = 0.0;
                current_color.g = 0.0;
                current_color.b = 1.0;
                current_color.a = 1.0;

                _MarkerMsg points_marker( points_marker_template_ );
                points_marker.header.stamp = now;
                points_marker.id = current_id ++;
                points_marker.color = current_color;

                for( auto joint = state->joints.begin(); joint != state->joints.end(); ++joint )
                {
                    // create point markers
                    points_marker.points.push_back( joint->pose.pose.position );
//                  points_marker.colors.push_back( current_color );

                    // map joint names to points for easy lookup later
                    point_map[joint->name] = joint->pose;
                }

                // connect points
                _MarkerMsg lines_marker( lines_marker_template_ );
                lines_marker.header.stamp = now;
                lines_marker.id = current_id ++;
                lines_marker.color = current_color;

                appendLineMarker( lines_marker, "head", "neck", point_map );
                appendLineMarker( lines_marker, "neck", "left_shoulder", point_map );
                appendLineMarker( lines_marker, "neck", "right_shoulder", point_map );
                appendLineMarker( lines_marker, "torso", "left_shoulder", point_map );
                appendLineMarker( lines_marker, "torso", "right_shoulder", point_map );
                appendLineMarker( lines_marker, "torso", "left_hip", point_map );
                appendLineMarker( lines_marker, "torso", "right_hip", point_map );
                appendLineMarker( lines_marker, "left_shoulder", "left_elbow", point_map );
                appendLineMarker( lines_marker, "left_elbow", "left_hand", point_map );
                appendLineMarker( lines_marker, "right_shoulder", "right_elbow", point_map );
                appendLineMarker( lines_marker, "right_elbow", "right_hand", point_map );
                appendLineMarker( lines_marker, "left_hip", "right_hip", point_map );
                appendLineMarker( lines_marker, "left_hip", "left_knee", point_map );
                appendLineMarker( lines_marker, "left_knee", "left_foot", point_map );
                appendLineMarker( lines_marker, "right_hip", "right_knee", point_map );
                appendLineMarker( lines_marker, "right_knee", "right_foot", point_map );

                // set text position
                _MarkerMsg text_marker( text_marker_template_ );
                text_marker.header.stamp = now;
                text_marker.id = current_id ++;
                text_marker.color = current_color;

                text_marker.text = state->name;
                text_marker.pose.position = point_map["torso"].pose.position;
                text_marker.pose.position.y -= 0.7;  // determined empirically...

                markers.markers.push_back( points_marker );
                markers.markers.push_back( lines_marker );
                markers.markers.push_back( text_marker );
            }
        }

        _HumanoidRecognizerPolicy::update( markers );

        auto & multi_pub = _HumanoidRecognizerPolicy::getMultiPub();
        multi_pub.publish( "humanoid_states_agg", quickdev::make_const_shared( combined_states_msg ) );

        // clear out the cache for the next update iteration
        state_arrays_cache.clear();
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
    {
        // get a lock to the cache
        QUICKDEV_TRY_LOCK_MUTEX( state_arrays_cache_ );
        // test the lock on the cache or return
        QUICKDEV_TRY_LOCK_OR_RETURN( state_arrays_cache_, "Dropping message [ %s ]", QUICKDEV_GET_MESSAGE_INST_NAME( msg ).c_str() );
        // if we got the lock, get a read/write reference to the cache's data
        auto & state_arrays_cache = state_arrays_cache_.get();

        state_arrays_cache.push_back( msg );
    }
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_
