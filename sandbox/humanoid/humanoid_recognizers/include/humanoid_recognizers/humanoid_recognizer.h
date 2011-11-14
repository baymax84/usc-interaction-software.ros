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
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>

#include <humanoid/humanoid.h>

#include <visualization_msgs/MarkerArray.h>

#include <deque>

QUICKDEV_DECLARE_NODE( HumanoidRecognizer )

using humanoid::_HumanoidStateArrayMsg;
using humanoid::_PoseWithConfidenceMsg;

QUICKDEV_DECLARE_NODE_CLASS( HumanoidRecognizer )
{
private:
	typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
	typedef visualization_msgs::Marker _MarkerMsg;

	ros::MultiSubscriber<> multi_sub_;
	ros::MultiPublisher<> multi_pub_;

	std::deque<_HumanoidStateArrayMsg::ConstPtr> state_arrays_cache_;

	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( HumanoidRecognizer )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		initAll();

		QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

		multi_pub_.addPublishers<_HumanoidStateArrayMsg, _MarkerArrayMsg>( nh_rel, { "humanoid_states", "/visualization_marker_array" } );
		multi_sub_.addSubscriber( nh_rel, "humanoid_states", &HumanoidRecognizerNode::humanoidStatesCB, this );
	}

	void appendLineMarker( _MarkerMsg & msg, const std::string & from, const std::string & to, const std::map<std::string, _PoseWithConfidenceMsg> & point_map )
	{
		const auto & from_point = point_map.find( from );
		const auto & to_point = point_map.find( to );

		if( from_point == point_map.end() || to_point == point_map.end() ) return;

		msg.points.push_back( from_point->second.pose.position );
		msg.points.push_back( to_point->second.pose.position );
	}

	QUICKDEV_SPIN_ONCE
	{
		if( state_arrays_cache_.size() == 0 ) return;

		_HumanoidStateArrayMsg combined_states_msg;
		_MarkerArrayMsg markers;

		_MarkerMsg marker_template;
		marker_template.header.stamp = ros::Time::now();
		marker_template.header.frame_id = "/primesensor/rgb";
		marker_template.ns = "basic_skeleton";
		marker_template.action = visualization_msgs::Marker::ADD;
		marker_template.lifetime = ros::Duration( 1.0 );
		marker_template.pose.orientation.w = 1.0;

		_MarkerMsg text_marker_template( marker_template );
		text_marker_template.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text_marker_template.scale.z = 0.1;

		_MarkerMsg lines_marker_template( marker_template );
		lines_marker_template.type = visualization_msgs::Marker::LINE_LIST;
		lines_marker_template.scale.x = 0.02;

		_MarkerMsg points_marker_template( marker_template );
		points_marker_template.type = visualization_msgs::Marker::SPHERE_LIST;
		points_marker_template.scale.x = points_marker_template.scale.y
									   = points_marker_template.scale.z
									   = 0.05;

		unsigned int current_id = 0;

		for( auto state_array = state_arrays_cache_.begin(); state_array != state_arrays_cache_.end(); ++state_array )
		{
			for( auto state = (*state_array)->states.begin(); state != (*state_array)->states.end(); ++state )
			{
				combined_states_msg.states.push_back( *state );

				std::map<std::string, _PoseWithConfidenceMsg> point_map;

				std_msgs::ColorRGBA current_color;
				current_color.r = 0.0;
				current_color.g = 0.0;
				current_color.b = 1.0;
				current_color.a = 1.0;

				_MarkerMsg points_marker( points_marker_template );
				points_marker.id = current_id ++;
				points_marker.color = current_color;

				for( auto joint = state->joints.begin(); joint != state->joints.end(); ++joint )
				{
					// create point markers
					points_marker.points.push_back( joint->pose.pose.position );

					// map joint names to points for easy lookup later
					point_map[joint->name] = joint->pose;
				}

				// connect points
				_MarkerMsg lines_marker( lines_marker_template );
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
				_MarkerMsg text_marker( text_marker_template );
				text_marker.id = current_id ++;
				text_marker.color = current_color;

				text_marker.text = state->name;
				text_marker.pose.position = point_map["torso"].pose.position;
				text_marker.pose.position.y -= 0.7l;  // determined empirically...

				markers.markers.push_back( points_marker );
				markers.markers.push_back( lines_marker );
				markers.markers.push_back( text_marker );
			}
		}

		multi_pub_.publish( "humanoid_states", quickdev::make_const_shared( combined_states_msg ) );
		multi_pub_.publish( "/visualization_marker_array", quickdev::make_const_shared( markers ) );

		state_arrays_cache_.clear();
	}

	QUICKDEV_DECLARE_MESSAGE_CALLBACK( humanoidStatesCB, _HumanoidStateArrayMsg )
	{
		state_arrays_cache_.push_back( msg );
	}
};

#endif // HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZERS_HUMANOIDRECOGNIZER_H_
