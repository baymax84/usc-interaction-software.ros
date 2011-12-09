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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_

#include <quickdev/node.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>

#include <deque>

typedef HumanoidRecognizerPolicy _HumanoidRecognizerPolicy;
QUICKDEV_DECLARE_NODE( HumanoidRecognizer, _HumanoidRecognizerPolicy )

using humanoid::_PoseWithConfidenceMsg;
using humanoid::_Humanoid;
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
        marker_template_.lifetime = ros::Duration( 0.1 );
        marker_template_.pose.orientation.w = 1.0;

        text_marker_template_ = marker_template_;
        //text_marker_template_.ns = "basic_skeleton_text";
        text_marker_template_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker_template_.scale.z = 0.1;

        lines_marker_template_ = marker_template_;
        //lines_marker_template_.ns = "basic_skeleton_lines";
        lines_marker_template_.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker_template_.scale.x = 0.02;

        points_marker_template_ = marker_template_;
        //points_marker_template_.ns = "basic_skeleton_points";
        points_marker_template_.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker_template_.scale.x = points_marker_template_.scale.y
                                        = points_marker_template_.scale.z
                                        = 0.05;
    }

    void appendLineMarker( _MarkerMsg & msg, const std::string & from, const std::string & to, const _Humanoid & humanoid )
    {
        const auto & from_point = humanoid.find( from );
        const auto & to_point = humanoid.find( to );

        if( from_point == humanoid.cend() || to_point == humanoid.cend() ) return;

        msg.points.push_back( from_point->pose.pose.position );
        msg.points.push_back( to_point->pose.pose.position );
    }

    QUICKDEV_SPIN_ONCE()
    {
        // get a read/write reference to the cache's data (called state_arrays_cache)
        QUICKDEV_LOCK_CACHE_AND_GET( state_arrays_cache_, state_arrays_cache );

        _HumanoidStateArrayMsg combined_states_msg;
        _MarkerArrayMsg markers;
        const auto now = ros::Time::now();

        unsigned int current_id = 0;

        for( auto users_components = state_arrays_cache.begin(); users_components != state_arrays_cache.end(); ++users_components )
        {
            // mesh all parts of all humanoids into one message
            _HumanoidRecognizerPolicy::updateHumanoids( *users_components );
        }

        // recursively erase all old messages
        _HumanoidRecognizerPolicy::eraseOld();

        auto & humanoids = _HumanoidRecognizerPolicy::getHumanoids();

        for( auto humanoid = humanoids.begin(); humanoid != humanoids.end(); ++humanoid )
        {
            //note: can't use cbegin() in for() above because getJointsMessage() modifies Humanoid (specifically, it modifies the ROS message cache of the humanoid's storage object)
            const auto & joints = humanoid->getJointsMessage();
            combined_states_msg.states.push_back( joints );

            //std::map<std::string, _PoseWithConfidenceMsg> point_map;

            std_msgs::ColorRGBA current_color;
            current_color.r = 0.0;
            current_color.g = 0.0;
            current_color.b = 1.0;
            current_color.a = 1.0;

            _MarkerMsg points_marker( points_marker_template_ );
            points_marker.header.stamp = now;
            points_marker.id = current_id ++;
            points_marker.color = current_color;

            for( auto joint = joints.joints.cbegin(); joint != joints.joints.cend(); ++joint )
            {
                // create point markers
                points_marker.points.push_back( joint->pose.pose.position );
//                  points_marker.colors.push_back( current_color );

                // map joint names to points for easy lookup later
                //point_map[joint->name] = joint->pose;
            }

            // connect points
            _MarkerMsg lines_marker( lines_marker_template_ );
            lines_marker.header.stamp = now;
            lines_marker.id = current_id ++;
            lines_marker.color = current_color;

            appendLineMarker( lines_marker, "head", "neck", *humanoid );
            appendLineMarker( lines_marker, "neck", "left_shoulder", *humanoid );
            appendLineMarker( lines_marker, "neck", "right_shoulder", *humanoid );
            appendLineMarker( lines_marker, "torso", "left_shoulder", *humanoid );
            appendLineMarker( lines_marker, "torso", "right_shoulder", *humanoid );
            appendLineMarker( lines_marker, "torso", "left_hip", *humanoid );
            appendLineMarker( lines_marker, "torso", "right_hip", *humanoid );
            appendLineMarker( lines_marker, "left_shoulder", "left_elbow", *humanoid );
            appendLineMarker( lines_marker, "left_elbow", "left_hand", *humanoid );
            appendLineMarker( lines_marker, "right_shoulder", "right_elbow", *humanoid );
            appendLineMarker( lines_marker, "right_elbow", "right_hand", *humanoid );
            appendLineMarker( lines_marker, "left_hip", "right_hip", *humanoid );
            appendLineMarker( lines_marker, "left_hip", "left_knee", *humanoid );
            appendLineMarker( lines_marker, "left_knee", "left_foot", *humanoid );
            appendLineMarker( lines_marker, "right_hip", "right_knee", *humanoid );
            appendLineMarker( lines_marker, "right_knee", "right_foot", *humanoid );

            // set text position
            _MarkerMsg text_marker( text_marker_template_ );
            text_marker.header.stamp = now;
            text_marker.id = current_id ++;
            text_marker.color = current_color;

            text_marker.text = humanoid->name;
            text_marker.pose.position = humanoid->at("head").pose.pose.position;
            text_marker.pose.position.z += 0.2;  // determined empirically...

            markers.markers.push_back( lines_marker );
            markers.markers.push_back( text_marker );
            markers.markers.push_back( points_marker );
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

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_
