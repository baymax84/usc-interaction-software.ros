/***************************************************************************
 *  visualization.h
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan J. Foster
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
 *  * Neither the name of rtk nor the names of its
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

#ifndef RSSDEMO_VISUALIZATION_H_
#define RSSDEMO_VISUALIZATION_H_

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

/// kinematics
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>
#include <rtk/math_impl.h>

typedef visualization_msgs::Marker      _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
typedef std_msgs::ColorRGBA _ColorMsg;
typedef geometry_msgs::Pose _PoseMsg;

using rtk::_FrameArray;


static int visualizeKDLChain(const KDL::Chain & chain , const KDL::JntArray & angles, const std::string & marker_namespace, _ColorMsg const & color)
{
  _FrameArray chain_pose;
  
  rtk::forwardKinematics(chain, angles, joint_position);
  
  visualizeHumanoidState(chain_pose, marker_namespace, color);
  
  return 0;
}

int visualizeFrameArray(_FrameArray pose, const std::string & marker_namespace, _ColorMsg const & color)
{
  ros::NodeHandle nh;
  
  _MarkerMsg 
    marker_template,
    joints_marker_template,
    text_marker_template,
    line_marker_template;
  
  _ColorMsg
    red_template,
    blue_template;
  
  std::string world_frame_name;
  nh.param<std::string>("world_frame_name", world_frame_name, "/world");
  
  red_template.a = 1.0;
  red_template.r = 1.0;
  red_template.b = 0.0;
  red_template.g = 0.0;
  blue_template.a = 1.0;
  blue_template.r = 0.0;
  blue_template.b = 1.0;
  blue_template.g = 0.0;
  
  marker_template.header.frame_id = world_frame_name_;
  
  marker_template.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_template.action = visualization_msgs::Marker::ADD;
  marker_template.lifetime = ros::Duration(1.0);
  marker_template.pose.orientation.w = 1.0;
  marker_template.color = red_template;
  marker_template.scale.x = 
    marker_template.scale.y = 
    marker_template.scale.z = .05;
  
  joints_marker_template = marker_template;
  joints_marker_template.type = visualization_msgs::Marker::SPHERE_LIST;
  /* joints_marker_template.scale.x =  */
  /* 	joints_marker_template.scale.y =  */
  /* 	joints_marker_template.scale.z = .05; */
  
  text_marker_template = marker_template;
  text_marker_template.type= visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker_template.scale.z = 0.1;
  
  line_marker_template = marker_template;
  line_marker_template.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker_template.scale.x = 0.02;
  
  auto now = ros::Time::now();
  
  _PoseMsg marker_pose;
  
  KDL::Frame pose_centroid = rtk::spatial::getCentroid(pose);
  
  _MarkerArrayMsg markers;
  _MarkerMsg joint_marker(joints_marker_template);
  _MarkerMsg text_marker(text_marker_template);
  _MarkerMsg chain_name_marker(text_marker_template);
  _MarkerMsg segment_marker(line_marker_template);
  _MarkerMsg ee_marker(marker_template);
  unsigned int id = 0;
  
  /* marker = joints_marker_template; */
  joint_marker.header.stamp = now;
  joint_marker.ns = marker_namespace;
  joint_marker.id = id++;
  joint_marker.color = color;
  
  text_marker.header.stamp = now;
  text_marker.ns = marker_namespace;
  text_marker.color = color;
  
  chain_name_marker.header.stamp = now;
  chain_name_marker.ns = marker_namespace;
  chain_name_marker.id = id++;
  chain_name_marker.color = color;
  
  segment_marker.header.stamp = now;
  segment_marker.ns = marker_namespace;
  segment_marker.id = id++;
  segment_marker.color = color;
  
    // TODO: Chamnge end effector marker to an arrow pointing along the segment to the end effector
    
    ee_marker.header.stamp = now;
    ee_marker.type = visualization_msgs::Marker::CUBE;
    ee_marker.ns = marker_namespace;
    ee_marker.id = id++;
    ee_marker.color = color;

    
    for(auto frame_it = pose.begin(); frame_it != pose.end(); ++frame_it, ++id)
      {
	tf::PoseKDLToMsg(*frame_it, marker_pose);
	
	joint_marker.points.push_back(marker_pose.position);
	segment_marker.points.push_back(marker_pose.position);
	
     }
    markers.markers.push_back(joint_marker);
    markers.markers.push_back(segment_marker);

    tf::PoseKDLToMsg(pose.back(), marker_pose);
    
    ee_marker.pose = marker_pose;
    
    markers.markers.push_back(ee_marker);

    chain_name_marker.text = marker_namespace;
    
    tf::PoseKDLToMsg(pose_centroid, marker_pose);
    chain_name_marker.pose = marker_pose;
    chain_name_marker.pose.position.z += 0.4;
    
    markers.markers.push_back(chain_name_marker);

        
    return 0;
  }
  

#endif // RSSDEMO_VISUALIZATION_H_
