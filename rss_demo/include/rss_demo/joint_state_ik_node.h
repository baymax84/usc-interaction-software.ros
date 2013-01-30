/***************************************************************************
 *  joint_state_ik_node.h
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

#ifndef RSSDEMO_JOINTSTATEIKNODE_H_
#define RSSDEMO_JOINTSTATEIKNODE_H_

#include <quickdev/node.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

/// data
#include <map>
/* #include <xmlrpcpp/XmlRpc.h> */

/// robot model
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>

/// kinematics
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <rtk/math_impl.h>

/// visualization
#include <visualization_msgs/MarkerArray.h>
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

typedef sensor_msgs::JointState _JointStateMsg;

typedef KDL::Joint _Joint;
typedef KDL::Segment _Segment;
typedef KDL::Vector _Vector;
typedef KDL::Rotation _Rotation;
typedef KDL::Frame _Frame;
typedef KDL::Chain _Chain;
typedef KDL::Tree _Tree;
typedef KDL::JntArray _JntArray;

using rtk::_FrameArray;

/// visualization-related typedefs
typedef visualization_msgs::Marker      _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;
typedef std_msgs::ColorRGBA _ColorMsg;
typedef geometry_msgs::Pose _PoseMsg;

typedef std::map<std::string, int> _JointOrderMap;

struct Retargeter
{
  std::string name_;
  _Frame target_to_source_;
  
  _Chain source_chain_;
  _Chain target_chain_;

  _JointOrderMap source_joint_order_map_;
  _JointOrderMap target_joint_order_map_;
  _JntArray target_state_;

  _JntArray target_limits_upper_;
  _JntArray target_limits_lower_;

  KDL::ChainFkSolverPos_recursive source_fk_;
  KDL::ChainFkSolverPos_recursive target_fk_;
  /* KDL::ChainIkSolverVel_pinv target_ik_vel_; */
  KDL::ChainIkSolverVel_wdls target_ik_vel_;
};

QUICKDEV_DECLARE_NODE( JointStateIK )

QUICKDEV_DECLARE_NODE_CLASS( JointStateIK )
{
 protected:
  /// communications
  ros::MultiPublisher<> multi_pub_;
  ros::MultiSubscriber<> multi_sub_;
  
  /// retargeting functionality
  /// Using deque instead of vector because std::vector requires that its storage type have a default constructor and an assignment operator
  std::deque<Retargeter> retargeters_;
  
    /// For visualization
  _ColorMsg
    red_template_,
    blue_template_;

  
  QUICKDEV_DECLARE_NODE_CONSTRUCTOR( JointStateIK )
  {
  }
  
  
  QUICKDEV_SPIN_FIRST()
    {
      QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

      srand(time(NULL));
      
      red_template_.a = 1.0;
      red_template_.r = 1.0;
      red_template_.b = 0.0;
      red_template_.g = 0.0;
      blue_template_.a = 1.0;
      blue_template_.r = 0.0;
      blue_template_.b = 1.0;
      blue_template_.g = 0.0;

      
      /// Initialize communications
      multi_sub_.addSubscriber( nh_rel, "source_joint_states",
				&JointStateIKNode::sourceJointStateCB, this );
      multi_sub_.addSubscriber( nh_rel, "target_joint_states",
				&JointStateIKNode::targetJointStateCB, this );
      multi_pub_.addPublishers<_JointStateMsg, _MarkerArrayMsg>( nh_rel, { "retargeted_joint_states", "visualization_marker_array" } );
      
      /// Get URDF path
      std::string source_urdf_path, target_urdf_path;
      
      if(!nh_rel.getParam("source_urdf", source_urdf_path))
	{
	  ROS_ERROR("Couldn't retrieve source urdf path from parameter server.");
	  exit(1);
	}
      if(!nh_rel.getParam("target_urdf", target_urdf_path))
	{
	  ROS_ERROR("Couldn't retrieve target urdf path from parameter server.");
	  exit(1);
	}
      
      /// Load the trees
      urdf::Model source_model, target_model;

      if(!source_model.initFile(source_urdf_path))
	{
	  ROS_ERROR("Failed to initialize source URDF ( %s ).", source_urdf_path.c_str());
	  exit(1);
	}
      else if (!target_model.initFile(target_urdf_path))
	{
	  ROS_ERROR("Failed to initialize target URDF ( %s ).", target_urdf_path.c_str());
	  exit(1);
	}
      else
	ROS_INFO("Initialized URDF files succesfully." );

      ROS_INFO("Loading kinematic trees...");
      _Tree source_tree, target_tree;
      
      if (!kdl_parser::treeFromUrdfModel(source_model, source_tree))
	{
	  ROS_ERROR("Failed to extract source tree from URDF model.");
	  exit(1);
	}
      if (!kdl_parser::treeFromUrdfModel(target_model, target_tree))
	{
	  ROS_ERROR("Failed to extract target tree from URDF model.");
	  exit(1);
	}
      
      /// Load chain endpoint data
      XmlRpc::XmlRpcValue chain_info;
      if(!nh_rel.getParam("chain_info", chain_info))
	{
	  ROS_ERROR("Couldn't retreive chain metadata from parameter server.");
	  exit(1);
	}
      
      /// TODO: find a better place to put this
      std::map<int, std::string> joint_type_text_map
	{
	  {urdf::Joint::UNKNOWN, "unknown"},
	  {urdf::Joint::REVOLUTE, "revolute"},
	  {urdf::Joint::CONTINUOUS, "continuous"},
	  {urdf::Joint::PRISMATIC, "prismatic"},
	  {urdf::Joint::FLOATING, "floating"},	
	  {urdf::Joint::PLANAR, "planar"},
	  {urdf::Joint::FIXED, "fixed"}
	};

      /// Get data for each chain that will be retargeted
      for(auto chain_it = chain_info.begin(); chain_it != chain_info.end(); ++chain_it)
	{
	  /// Chains that we will be using for retargeting
	  _Chain source_chain, target_chain;

	  /// The name of the chain being retargeted
	  std::string chain_name = chain_it->first;
	  XmlRpc::XmlRpcValue & source_endpoints = chain_it->second["source"];
	  XmlRpc::XmlRpcValue & target_endpoints = chain_it->second["target"];
	  XmlRpc::XmlRpcValue & t2s_xml = chain_it->second["target_to_source"];
	  
	  _Frame target_to_source = _Frame(_Rotation::RPY(t2s_xml["roll"], t2s_xml["pitch"], t2s_xml["yaw"]),
					   _Vector(t2s_xml["x"], t2s_xml["y"], t2s_xml["z"]));
	  
	  ROS_INFO_STREAM( chain_name << " endpoints: " );
	  ROS_INFO_STREAM( "source:");
	  ROS_INFO_STREAM( "\t" << source_endpoints["begin"]
			   << " ----> "
			   << source_endpoints["end"]);
	  ROS_INFO_STREAM( "target:" );
	  ROS_INFO_STREAM( target_endpoints["begin"]
			   << " ----> "
			   << target_endpoints["end"] );
	  ROS_INFO("--------------------");
	  
	  if( !source_tree.getChain(source_endpoints["begin"], source_endpoints["end"], source_chain) )
	    {
	      ROS_WARN("Failed to extract source chain for ( %s ). Chain will not be retargeted.", chain_name.c_str());
	      continue;
	    }
	  printChainVerbose(source_chain, chain_name + "_source");
	  
	  if( !target_tree.getChain(target_endpoints["begin"], target_endpoints["end"], target_chain) )
	    {
	      ROS_WARN("Failed to extract target chain for ( %s ). Chain will not be retargeted.", chain_name.c_str());
	      continue;
	    }
	  printChainVerbose(target_chain, chain_name + "_target");
	  
	  /// Get the order in which the joints occur in the chain, starting with the base joint. This is used to get data from joint state messages
	  _JointOrderMap source_joint_order_map;
	  int order = 0;
	  for(auto segment_it = source_chain.segments.begin(); segment_it != source_chain.segments.end(); ++segment_it)
	    {
	      _Joint const & joint = segment_it->getJoint();
	      
	      if( joint.getType() != _Joint::None )
		{
		  source_joint_order_map[joint.getName()] = order++;
		}
	    }

	  _JointOrderMap target_joint_order_map;
	  order = 0;
	  for(auto segment_it = target_chain.segments.begin(); segment_it != target_chain.segments.end(); ++segment_it)
	    {
	      _Joint const & joint = segment_it->getJoint();
	      
	      if( joint.getType() != _Joint::None )
		{
		  target_joint_order_map[joint.getName()] = order++;
		}
	    }

	  /// TIME TO LOAD THE JOINT LIMITS FOR THE TARGET! (We don't actually use these right now)
	  /// Number excludes fixed joints
	  unsigned int nr_target_joints = target_chain.getNrOfJoints();
	  _JntArray target_limits_upper(nr_target_joints), 
	    target_limits_lower(nr_target_joints);
	  
	  
	  ROS_INFO_STREAM( chain_name << " target joint limits: ");
	  
	  boost::shared_ptr<const urdf::Link> link = target_model.getLink(target_endpoints["end"]);
      	  boost::shared_ptr<const urdf::Joint> joint = target_model.getJoint(link->parent_joint->name);
	  
	  int i = 0;
      	  while(link->name != std::string(target_endpoints["begin"]))
      	    {
	      /**
	       * We don't apply joint angles to fixed joints when we retarget, so we exclude these.
	       * It's also worth noting that the KDL FK solvers do not accept joint angles for fixed joints.
	       */
      	      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      		{
      		  double upper,lower;
		  
      		  joint = target_model.getJoint(link->parent_joint->name);
		  
      		  // get limits
      		  if (joint->type != urdf::Joint::CONTINUOUS)
      		    {
      		      lower = joint->limits->lower;
      		      upper = joint->limits->upper;
      		    }
      		  else
      		    {
      		      lower = 0.0;
      		      upper = 0.0;
      		    }
	    
      		  ROS_INFO_STREAM( '\t' << joint->name.c_str() << "( " << joint_type_text_map[joint->type] << " ): " << joint->limits->lower << " to " << joint->limits->upper );
	      
      		  // Add limits to joint array
		  target_limits_upper(i) = upper;
		  target_limits_lower(i) = lower;
		  ++i;
		}
      	      // jump one link lower in the chain
      	      link = target_model.getLink(link->getParent()->name);
      	    }
	  
	  /// normalize the target chain to 1 meter
	  /* target_chain = rtk::spatial::normalize(target_chain); */
	  
	  /// Let's initialize the retargeter using our good friend aggregate initialization.
	  Retargeter retargeter
	  {
	    chain_name, target_to_source, source_chain, 
	      target_chain, source_joint_order_map, 
	      target_joint_order_map, _JntArray(target_chain.getNrOfJoints()),
	      target_limits_upper, target_limits_lower,
	      KDL::ChainFkSolverPos_recursive(source_chain),
	      KDL::ChainFkSolverPos_recursive(target_chain),
	      /* KDL::ChainIkSolverVel_pinv(target_chain) */
	      KDL::ChainIkSolverVel_wdls(target_chain, .01, 10000)
	      };
	  
	  retargeters_.push_back(retargeter);
	}

      if ( !(retargeters_.size()) > 0 )
	{
	  ROS_WARN("No retargeters could be loaded successfully.");
	}
 
      initPolicies<quickdev::policy::ALL>();
    }
  
  
  QUICKDEV_SPIN_ONCE()
    {
      ROS_INFO("IK Node is spinning...");
    }

  QUICKDEV_DECLARE_MESSAGE_CALLBACK2( targetJointStateCB, _JointStateMsg, joint_state_msg )
    {
      ROS_INFO("Got a bandit joint state..." );
      
      std::vector<std::string> const & names = joint_state_msg->name;
      std::vector<double> const & positions = joint_state_msg->position;
     
      for(auto retargeter_it = retargeters_.begin(); retargeter_it != retargeters_.end(); ++retargeter_it)
	{
	  _JointOrderMap const & joint_order_map = retargeter_it->target_joint_order_map_;
	  _JntArray & target_state = retargeter_it->target_state_;
	  
	  int nr_target_joints = retargeter_it->target_chain_.getNrOfJoints();
	  
	  target_state = _JntArray(nr_target_joints);
	  
	  int nr_matches = 0;
	  auto name_it = names.begin();
	  for(auto position_it = positions.begin(); position_it != positions.end(); ++position_it, ++name_it)
	    {
	      /// Check if the current joint is part of this chain
	      auto match_it = joint_order_map.find( *name_it );
	      
	      if ( match_it != joint_order_map.end() )
		{
		  target_state( match_it->second ) = *position_it;
		  ++nr_matches;
		}
	    }
	  if( nr_matches != nr_target_joints )
	    target_state = _JntArray(nr_target_joints);
	}
    }

  /// joint_state_msg is a boost::shared_ptr to a JointState message
  QUICKDEV_DECLARE_MESSAGE_CALLBACK2( sourceJointStateCB, _JointStateMsg, joint_state_msg )
    {
      ros::Time now = ros::Time::now();
      
      std::vector<std::string> const & names = joint_state_msg->name;
      std::vector<double> const & positions = joint_state_msg->position;

      /// The message with the retargeted chain joint state that we will publish
      _JointStateMsg retargeted_joint_state;
      int nr_success = 0;

      /// Time to retarget
      for(auto retargeter_it = retargeters_.begin(); retargeter_it != retargeters_.end(); ++retargeter_it)
	{
	  
	  /* if( retargeter_it->target_state_.rows() == 0 ) */
	  /*   continue; */
	  
	  
	  std::string const & chain_name = retargeter_it->name_;
	  _JointOrderMap const & joint_order_map = retargeter_it->source_joint_order_map_;
	  
	  _Chain const & source_chain = retargeter_it->source_chain_;
	  /* _Chain const target_chain = rtk::spatial::normalize(retargeter_it->target_chain_); */
	  _Chain const & target_chain = retargeter_it->target_chain_;
	  ROS_INFO( "Normalized length for target chain is %f.", rtk::spatial::getLength(target_chain) );
	  int nr_source_joints = source_chain.getNrOfJoints();
	  _JntArray source_state( nr_source_joints );

	  ROS_INFO("Solving [ %s ]", chain_name.c_str());
	  
	  /// Extract the joint angles corresponding to this chain
	  int nr_matches = 0;
	  auto name_it = names.begin();
	  for(auto position_it = positions.begin(); position_it != positions.end(); ++position_it, ++name_it)
	    {
	      /// Check if the current joint is part of this chain
	      auto match_it = joint_order_map.find( *name_it );
	      
	      if ( match_it != joint_order_map.end() )
		{
		  source_state( match_it->second ) = *position_it;
		  ++nr_matches;
		}
	    }
	  if( nr_matches != nr_source_joints )
	    {
	      ROS_WARN("Incoming joint state did not fully specify chain state.");
	      ROS_WARN("Joints in chain: %d, Matching joints received: %d", 
		       nr_source_joints, nr_matches);
	      ROS_WARN("Skipping chain...");
	      continue;
	    }
	  
	  _FrameArray source_frames;
	  if( rtk::spatial::forwardKinematics(source_chain, source_state, source_frames) )
	    {
	      ROS_WARN("Foward kinematics failed.");
	      ROS_WARN("Skipping chain...");
	      continue;
	    }
	  
	  /* source_frames = rtk::spatial::normalize(source_frames); */

	  /// TODO: Figure out why adding chain scaling broke program
	  rtk::spatial::transform(source_frames, retargeter_it->target_to_source_);

	  _JntArray target_zero(target_chain.getNrOfJoints());
	  _FrameArray target_straight;
	  rtk::spatial::forwardKinematics(target_chain, target_zero, target_straight);
	  
	  double source_length = rtk::spatial::getLength(source_frames);
	  double target_length = rtk::spatial::getLength(target_straight);
	  double target_to_source_ratio = target_length / source_length;

	  source_frames = rtk::spatial::scaleChain(source_frames, target_to_source_ratio);
	  
	  /// The IK solver we will use
	  /// TODO: Get these values (and maybe retargeting algorithm) from config
	  /* KDL::ChainIkSolverPos_NR_JL ik_solver(target_chain, retargeter_it->target_limits_lower_, retargeter_it->target_limits_upper_, */
	  /* 					retargeter_it->target_fk_, retargeter_it->target_ik_vel_, 10000, 0.01); */

	  KDL::ChainIkSolverPos_NR ik_solver(target_chain, retargeter_it->target_fk_, retargeter_it->target_ik_vel_, 10000, 0.0000001);
	  
	  int nr_joints = target_chain.getNrOfJoints();
	  _JntArray ik_in(nr_joints), ik_out(nr_joints);
	  
	  
	  /// Attempt to provide a non-singular initial joint state
	  for(int i = 0 ; i < nr_joints; ++i)
	    {
	      ik_in(i) = rand() % 100 / (M_PI * 100);
	    }
	  
	  
	  _Frame ee_frame = source_frames.back();
	  ROS_INFO("EE frame is (%f, %f, %f)", ee_frame.p.x(), ee_frame.p.y(), ee_frame.p.z());
	  
	  _MarkerMsg ee_marker;
	  _MarkerArrayMsg markers;
	  ee_marker.header.frame_id = "/world";
	  ee_marker.header.stamp = now;
	  ee_marker.ns = "end_effector";
	  ee_marker.type = visualization_msgs::Marker::SPHERE;
	  ee_marker.action = visualization_msgs::Marker::ADD;
	  ee_marker.lifetime = ros::Duration(1.0);
	  tf::PoseKDLToMsg(ee_frame, ee_marker.pose);
	  /* ee_marker.pose.orientation.w = 1.0; */
	  ee_marker.color = blue_template_;
	  ee_marker.scale.x = 
	    ee_marker.scale.y = 
	    ee_marker.scale.z = 0.1;
	  markers.markers.push_back(ee_marker);

	  multi_pub_.publish("visualization_marker_array", markers);


	  
	  /// Scale the desired end effector pose so that the target chain can reach it
	  /* ee_frame.p = ee_frame.p * target_to_source_ratio; */

	  std::string ee_name = (target_chain.segments.end() -1)->getJoint().getName();
	  ROS_INFO("Using end effector %s", ee_name.c_str());

	  int ik_error = ik_solver.CartToJnt(ik_in, ee_frame, ik_out);
	  /* int ik_error = ik_solver.CartToJnt(retargeter_it->target_state_, ee_frame, ik_out); */

#if RSSDEMO_JOINTSTATEIKNODE_DEBUG
	  ROS_INFO("visualizing source chain...");
	  visualizeFrameArray( source_frames, chain_name + "_source_normalized", blue_template_);
	  /* visualizeFrameArray( rtk::spatial::scaleChain(source_frames, target_to_source_ratio), */
	  /* 		       chain_name + "_source_scaled", blue_template_); */

	  /* _JntArray target_zero(target_chain.getNrOfJoints()); */
	  
	  ROS_INFO("visualizing untargeted chain...");
	  visualizeKDLChain(target_chain, target_zero, chain_name + "_untargeted_normalized", red_template_);

	  _FrameArray retargeted_frames;
	  rtk::spatial::forwardKinematics(target_chain, ik_out, retargeted_frames);

	  ROS_INFO("visualizing retargeted chain...");
	  visualizeFrameArray(retargeted_frames, chain_name + "_retargeted_normalized", red_template_);
	  /* visualizeFrameArray(rtk::spatial::normalize(retargeted_frames), */
	  /* 		      chain_name + "_retargeted_normalized", red_template_); */
	  /// visualize all of the chains involved in this algorithm
#endif	  


	  if( ik_error )
	    {
	      ROS_WARN("IK Solver failed (error code %d).", ik_error);
	      /* ROS_WARN("Skipping chain..."); */
	      /* continue; */
	    }
	  
	  for(unsigned int i = 0; i < target_chain.getNrOfSegments(); ++i)
	    {
	      _Joint const & joint = target_chain.getSegment(i).getJoint();
	      
	      if( joint.getType() != _Joint::None )
		{
		  retargeted_joint_state.name.push_back(joint.getName());
		  retargeted_joint_state.position.push_back(ik_out(i));
		}
	    }

	  ++nr_success;
	}

      if(nr_success > 0)
	{
	  multi_pub_.publish("retargeted_joint_states", retargeted_joint_state);
	  ROS_INFO( "Publishing joint states for %d chains.", nr_success );
	}
      
      return;
    }
  
 private:
  
  void printChainVerbose(_Chain const & chain, std::string const & name)
  {
    ROS_INFO_STREAM( "Loaded " << name << " :" );
    ROS_INFO_STREAM( "\tSegments: " << chain.getNrOfSegments() );
    ROS_INFO_STREAM( "\tJoints (excluding Joint::NONE): " << chain.getNrOfJoints() );
        
    for(auto segment_it = chain.segments.begin(); segment_it != chain.segments.end(); ++segment_it)
      {
	_Joint const & joint = segment_it->getJoint();
	_Frame const & f2t   = segment_it->getFrameToTip();
	ROS_INFO_STREAM( '\t' << joint.getName() << " (" <<  joint.getTypeName() << ") -> " 
			 << segment_it->getName() << " ( " << f2t.p.x() << ", " << f2t.p.y() 
			 << ", " << f2t.p.z() << " )" );
      }
  }
  
  int visualizeKDLChain(const KDL::Chain & chain , const KDL::JntArray & angles, const std::string & marker_namespace, _ColorMsg const & color)
  {
    _FrameArray chain_pose;
  
    rtk::spatial::forwardKinematics(chain, angles, chain_pose);
  
    visualizeFrameArray(chain_pose, marker_namespace, color);
  
    return 0;
  }

  int visualizeFrameArray(_FrameArray pose, const std::string & marker_namespace, _ColorMsg const & color)
  {
    QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
  
    _MarkerMsg 
      marker_template,
      joints_marker_template,
      text_marker_template,
      line_marker_template;
  

  
    std::string world_frame_name;
    nh_rel.param<std::string>("world_frame_name", world_frame_name, "/world");
  
    /* red_template.a = 1.0; */
    /* red_template.r = 1.0; */
    /* red_template.b = 0.0; */
    /* red_template.g = 0.0; */
    /* blue_template.a = 1.0; */
    /* blue_template.r = 0.0; */
    /* blue_template.b = 1.0; */
    /* blue_template.g = 0.0 */;
  
    marker_template.header.frame_id = world_frame_name;
  
    marker_template.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_template.action = visualization_msgs::Marker::ADD;
    marker_template.lifetime = ros::Duration(1.0);
    marker_template.pose.orientation.w = 1.0;
    marker_template.color = color;
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
    
    multi_pub_.publish("visualization_marker_array", markers );
        
    return 0;
  }

};

#endif // RSSDEMO_JOINTSTATEIKNODE_H_
