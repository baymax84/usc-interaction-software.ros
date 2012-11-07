/***************************************************************************
 *  include/humanoid_recognizers/humanoid_aggregator_node.h
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

#ifndef HUMANOIDRECOGNIZERS_HUMANOIDAGGREGATORNODE_H_
#define HUMANOIDRECOGNIZERS_HUMANOIDAGGREGATORNODE_H_

#include <quickdev/node.h>

#include <humanoid_recognizers/humanoid_recognizer_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

#include <deque>

using humanoid::_PoseWithConfidenceMsg;
using humanoid::_Humanoid;
using humanoid::_HumanoidStateArrayMsg;
using humanoid::_JointStateMsg;
using humanoid::_JointStateArrayMsg;

typedef HumanoidRecognizerPolicy<_HumanoidStateArrayMsg> _HumanoidRecognizerPolicy;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

QUICKDEV_DECLARE_NODE( HumanoidAggregator, _HumanoidRecognizerPolicy, _TfTranceiverPolicy )

typedef _HumanoidRecognizerPolicy::_MarkerArrayMsg _MarkerArrayMsg;
typedef _HumanoidRecognizerPolicy::_MarkerMsg _MarkerMsg;

QUICKDEV_DECLARE_NODE_CLASS( HumanoidAggregator )
{
private:
    quickdev::MutexedCache<std::deque<_HumanoidStateArrayMsg::ConstPtr> > state_arrays_cache_;

    std::string world_frame_name_;

    _MarkerMsg
        marker_template_,
        text_marker_template_,
        lines_marker_template_,
        arrow_marker_template_,
        points_marker_template_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( HumanoidAggregator )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        //initAll( "features_topic_name_param", std::string( "humanoid_states_agg" ) );
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        //multi_pub_.addPublishers<_JointStateArrayMsg>( nh_rel_, { "joint_states" } );

        _HumanoidRecognizerPolicy::registerCallback( quickdev::auto_bind( &HumanoidAggregatorNode::humanoidStatesCB, this ) );

        initPolicies<_HumanoidRecognizerPolicy>( "update_humanoids_param", false );

        initPolicies<quickdev::policy::ALL>();

        world_frame_name_ = quickdev::ParamReader::readParam<decltype( world_frame_name_ )>( nh_rel, "world_frame_name", "/world" );

        marker_template_.header.frame_id = world_frame_name_;
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

        arrow_marker_template_ = marker_template_;
        arrow_marker_template_.type = visualization_msgs::Marker::ARROW;
        arrow_marker_template_.scale.x = 0.25;
        arrow_marker_template_.scale.y = 0.3;
        arrow_marker_template_.scale.z = 0.3;

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

        auto const now = ros::Time::now();

        //printf( "cached %zu humanoid messages\n", state_arrays_cache.size() );

        // given the desired "world" frame, transform all the joints of all incoming humanoids to this world frame, then mesh them together into one message
        for( auto users_msg_it = state_arrays_cache.begin(); users_msg_it != state_arrays_cache.end(); ++users_msg_it )
        {
            // copy
            _HumanoidStateArrayMsg users_msg = *(*users_msg_it);
            auto & users = users_msg.states;

            for( auto user_it = users.begin(); user_it != users.end(); ++user_it )
            {
                auto & joints = user_it->joints;

                for( auto joint_it = joints.begin(); joint_it != joints.end(); ++joint_it )
                {
                    auto & joint = *joint_it;

                    // if the joint was already measured with respect to the world frame, then move to the next joint
                    if( joint.header.frame_id == world_frame_name_ ) continue;

                    // otherwise, transform the joint into the coordinate frame specified

                    // the joint was measured with respect to some sensor; get that transform
                    auto const sensor_to_joint_tf = unit::convert<btTransform>( joint.pose.pose );
                    // the sensor should exist somewhere in the world; look up that transform
                    auto const world_to_sensor_tf = _TfTranceiverPolicy::waitForAndLookupTransform( world_frame_name_, joint.header.frame_id, 10, 10 );

                    joint.pose.pose = unit::implicit_convert( world_to_sensor_tf * sensor_to_joint_tf );
                    joint.header.frame_id = world_frame_name_;
                }
            }

            // mesh all parts of all humanoids into one message
            _HumanoidRecognizerPolicy::updateHumanoids( users_msg );
        }

        /*std::cout << "------------" << std::endl;
        auto humanoids_temp = _HumanoidRecognizerPolicy::getHumanoids();
        for( auto humanoid = humanoids_temp.begin(); humanoid != humanoids_temp.end(); ++humanoid )
        {
            std::cout << "Tracking: " << humanoid->name << std::endl;
        }*/

        // recursively erase all old messages
        _HumanoidRecognizerPolicy::eraseOld( 2.0 );

//        quickdev::start_stream_indented( "getting a copy of the list of humanoids from _HumanoidRecognizerPolicy" );
        auto & humanoids = _HumanoidRecognizerPolicy::getHumanoids();
//        quickdev::end_stream_indented();

        PRINT_INFO( "tracking %zu humanoids", humanoids.size() );

        _MarkerArrayMsg markers;
        unsigned int current_id = 0;

        _HumanoidStateArrayMsg combined_states_msg;
        combined_states_msg.states.reserve( humanoids.size() );

        //_JointStateArrayMsg joint_states_msg;
        //joint_states_msg.names.reserve( humanoids.size() );
        //joint_states_msg.states.reserve( humanoids.size() );

        for( auto humanoid_it = humanoids.begin(); humanoid_it != humanoids.end(); ++humanoid_it )
        {
            auto & humanoid = *humanoid_it;
            //joint_states_msg.names.push_back( humanoid.name );
            //joint_states_msg.states.push_back( humanoid.getJointStateMessage() );

            //note: can't use cbegin() in for() above because getJointsMessage() modifies Humanoid (specifically, it modifies the ROS message cache of the humanoid's storage object)
            const auto & joints = humanoid.getJointsMessage();

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
                points_marker.points.push_back( joint->pose.pose.position );
            }

            // connect points
            _MarkerMsg lines_marker( lines_marker_template_ );
            lines_marker.header.stamp = now;
            lines_marker.id = current_id ++;
            lines_marker.color = current_color;

            appendLineMarker( lines_marker, "head", "neck", humanoid );
            appendLineMarker( lines_marker, "neck", "left_shoulder", humanoid );
            appendLineMarker( lines_marker, "neck", "right_shoulder", humanoid );
            appendLineMarker( lines_marker, "torso", "left_shoulder", humanoid );
            appendLineMarker( lines_marker, "torso", "right_shoulder", humanoid );
            appendLineMarker( lines_marker, "torso", "left_hip", humanoid );
            appendLineMarker( lines_marker, "torso", "right_hip", humanoid );
            appendLineMarker( lines_marker, "left_shoulder", "left_elbow", humanoid );
            appendLineMarker( lines_marker, "left_elbow", "left_hand", humanoid );
            appendLineMarker( lines_marker, "right_shoulder", "right_elbow", humanoid );
            appendLineMarker( lines_marker, "right_elbow", "right_hand", humanoid );
            appendLineMarker( lines_marker, "left_hip", "right_hip", humanoid );
            appendLineMarker( lines_marker, "left_hip", "left_knee", humanoid );
            appendLineMarker( lines_marker, "left_knee", "left_ankle", humanoid );
            appendLineMarker( lines_marker, "left_ankle", "left_foot", humanoid );
            appendLineMarker( lines_marker, "right_hip", "right_knee", humanoid );
            appendLineMarker( lines_marker, "right_knee", "right_ankle", humanoid );
            appendLineMarker( lines_marker, "right_ankle", "right_foot", humanoid );
            appendLineMarker( lines_marker, "head", "eyes", humanoid );

            _MarkerMsg eyes_arrow_marker = arrow_marker_template_;
            eyes_arrow_marker.pose = humanoid["eyes"].pose.pose;
            eyes_arrow_marker.id = current_id ++;
            eyes_arrow_marker.color = current_color;
            markers.markers.push_back( eyes_arrow_marker );

            _MarkerMsg head_arrow_marker = arrow_marker_template_;
            head_arrow_marker.pose = humanoid["head"].pose.pose;
            head_arrow_marker.id = current_id ++;
            head_arrow_marker.color = current_color;
            markers.markers.push_back( head_arrow_marker );

            _MarkerMsg neck_arrow_marker = arrow_marker_template_;
            neck_arrow_marker.pose = humanoid["neck"].pose.pose;
            neck_arrow_marker.id = current_id ++;
            neck_arrow_marker.color = current_color;
            markers.markers.push_back( neck_arrow_marker );

            _MarkerMsg torso_arrow_marker = arrow_marker_template_;
            torso_arrow_marker.pose = humanoid["torso"].pose.pose;
            torso_arrow_marker.id = current_id ++;
            torso_arrow_marker.color = current_color;
            markers.markers.push_back( torso_arrow_marker );

            _MarkerMsg pelvis_arrow_marker = arrow_marker_template_;
            pelvis_arrow_marker.pose = humanoid["pelvis"].pose.pose;
            pelvis_arrow_marker.id = current_id ++;
            pelvis_arrow_marker.color = current_color;
            markers.markers.push_back( pelvis_arrow_marker );

            _MarkerMsg l_ear_arrow_marker = arrow_marker_template_;
            l_ear_arrow_marker.pose = humanoid["left_ear"].pose.pose;
            l_ear_arrow_marker.id = current_id ++;
            l_ear_arrow_marker.color = current_color;
            markers.markers.push_back( l_ear_arrow_marker );

            _MarkerMsg r_ear_arrow_marker = arrow_marker_template_;
            r_ear_arrow_marker.pose = humanoid["right_ear"].pose.pose;
            r_ear_arrow_marker.id = current_id ++;
            r_ear_arrow_marker.color = current_color;
            markers.markers.push_back( r_ear_arrow_marker );

            // set text position
            _MarkerMsg text_marker( text_marker_template_ );
            text_marker.header.stamp = now;
            text_marker.id = current_id ++;
            text_marker.color = current_color;

            text_marker.text = humanoid.name;
            text_marker.pose.position = humanoid["head"].pose.pose.position;
            text_marker.pose.position.z += 0.2;  // determined empirically...

            markers.markers.push_back( lines_marker );
            markers.markers.push_back( text_marker );
            markers.markers.push_back( points_marker );
        }

         _HumanoidRecognizerPolicy::updateFeatures( combined_states_msg );
         _HumanoidRecognizerPolicy::updateMarkers( markers );

        //multi_pub_.publish( "joint_states", joint_states_msg );

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

#endif // HUMANOIDRECOGNIZERS_HUMANOIDAGGREGATORNODE_H_
